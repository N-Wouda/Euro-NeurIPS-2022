#include "Individual.h"
#include "Params.h"

#include <cassert>
#include <deque>
#include <fstream>
#include <numeric>
#include <vector>

namespace
{
// Structure representing a client when making routes
struct ClientSplit
{
    int demand = 0;       // The demand of the client
    int serviceTime = 0;  // The service duration of the client
    int d0_x = 0;         // The distance from the depot to the client
    int dx_0 = 0;         // The distance from the client to the depot
    int dnext = 0;        // The distance from the client to the next client
};

struct ClientSplits
{
    int vehicleCap;
    double capPenalty;

    std::vector<ClientSplit> splits;
    std::vector<int> predecessors;
    std::vector<double> pathCosts;

    std::vector<int> cumDist;
    std::vector<int> cumLoad;
    std::vector<int> cumServ;

    explicit ClientSplits(Params const &params, std::vector<int> const &tour)
        : vehicleCap(params.vehicleCapacity),
          capPenalty(params.penaltyCapacity),
          splits(params.nbClients + 1),
          predecessors(params.nbClients + 1, 0),
          pathCosts(params.nbClients + 1, 1.e30),
          cumDist(params.nbClients + 1, 0),
          cumLoad(params.nbClients + 1, 0),
          cumServ(params.nbClients + 1, 0)
    {
        pathCosts[0] = 0;

        for (int idx = 1; idx <= params.nbClients; idx++)  // exclude depot
        {
            auto const curr = tour[idx - 1];
            auto const dist = idx < params.nbClients  // INT_MIN for last client
                                  ? params.timeCost.get(curr, tour[idx])
                                  : INT_MIN;

            splits[idx] = {params.cli[curr].demand,
                           params.cli[curr].serviceDuration,
                           params.timeCost.get(0, curr),
                           params.timeCost.get(curr, 0),
                           dist};

            cumLoad[idx] = cumLoad[idx - 1] + splits[idx].demand;
            cumServ[idx] = cumServ[idx - 1] + splits[idx].serviceTime;
            cumDist[idx] = cumDist[idx - 1] + splits[idx - 1].dnext;
        }
    }

    // Computes the cost of propagating label i to j
    [[nodiscard]] inline double propagate(int i, int j) const
    {
        assert(i < j);
        auto const excessCapacity = cumLoad[j] - cumLoad[i] - vehicleCap;
        auto const deltaDist = cumDist[j] - cumDist[i + 1];
        return pathCosts[i] + deltaDist + splits[i + 1].d0_x + splits[j].dx_0
               + capPenalty * std::max(excessCapacity, 0);
    }

    // Tests if i dominates j as a predecessor for all nodes x >= j + 1
    [[nodiscard]] inline bool leftDominates(int i, int j) const
    {
        assert(i < j);
        auto const lhs = pathCosts[j] + splits[j + 1].d0_x;
        auto const deltaDist = cumDist[j] - cumDist[i + 1];
        auto const rhs = pathCosts[i] + splits[i + 1].d0_x + deltaDist
                         + capPenalty * (cumLoad[j] - cumLoad[i]);

        return lhs + MY_EPSILON > rhs;
    }

    // Tests if j dominates i as a predecessor for all nodes x >= j + 1
    [[nodiscard]] inline bool rightDominates(int i, int j) const
    {
        assert(i < j);
        auto const lhs = pathCosts[j] + splits[j + 1].d0_x;
        auto const rhs = pathCosts[i] + splits[i + 1].d0_x + cumDist[j + 1]
                         - cumDist[i + 1];

        return lhs < rhs + MY_EPSILON;
    };
};
}  // namespace

void Individual::makeRoutes()
{
    ClientSplits splits(*params, getTour());

    auto deq = std::deque<int>(params->nbClients + 1);
    deq.push_front(0);  // depot

    for (int idx = 1; idx <= params->nbClients; idx++)  // exclude depot
    {
        splits.pathCosts[idx] = splits.propagate(deq.front(), idx);
        splits.predecessors[idx] = deq.front();  // best predecessor for idx

        if (idx == params->nbClients)
            break;

        // idx will be inserted if idx is not dominated by the last client: we
        // need to remove whoever is dominated by idx.
        if (!splits.leftDominates(deq.back(), idx))
        {
            while (!deq.empty() && splits.rightDominates(deq.back(), idx))
                deq.pop_back();

            deq.push_back(idx);
        }

        while (deq.size() >= 2)  // Check if the current front is dominated by
        {                        // the follow-up client. If so, remove.
            auto const firstProp = splits.propagate(deq[0], idx + 1);
            auto const secondProp = splits.propagate(deq[1], idx + 1);

            if (firstProp + MY_EPSILON > secondProp)
                deq.pop_front();
            else
                break;
        }
    }

    if (splits.pathCosts[params->nbClients] > 1.e29)  // has not been updated
        throw std::runtime_error("No split solution reached the last client");

    int end = params->nbClients;
    for (auto &route : routeChrom)
    {
        route.clear();

        if (end != 0)  // assign routes
        {
            int begin = splits.predecessors[end];
            route = std::vector<Client>(tourChrom.begin() + begin,
                                        tourChrom.begin() + end);

            end = begin;
        }
    }

    evaluateCompleteCost();
}

void Individual::evaluateCompleteCost()
{
    costs = CostSol();

    for (auto const &route : routeChrom)
    {
        costs.nbRoutes++;

        if (route.empty())  // First empty route. Due to makeRoutes() all
            break;          // subsequent routes are empty as well

        int maxReleaseTime = 0;
        for (auto const idx : route)
            maxReleaseTime
                = std::max(maxReleaseTime, params->cli[idx].releaseTime);

        // Get the distance, load, serviceDuration and time associated with the
        // vehicle traveling from the depot to the first client. Assume depot
        // has service time 0 and earliestArrival 0
        int distance = params->timeCost.get(0, route[0]);
        int load = params->cli[route[0]].demand;

        int time = maxReleaseTime + distance;
        int waitTime = 0;
        int timeWarp = 0;

        if (time < params->cli[route[0]].earliestArrival)
        {
            // Don't add wait time since we can start route later (doesn't
            // really matter since there is no penalty anyway).
            time = params->cli[route[0]].earliestArrival;
        }

        // Add possible time warp
        if (time > params->cli[route[0]].latestArrival)
        {
            timeWarp += time - params->cli[route[0]].latestArrival;
            time = params->cli[route[0]].latestArrival;
        }

        predecessors[route[0]] = 0;

        // Loop over all clients for this vehicle
        for (size_t idx = 1; idx < route.size(); idx++)
        {
            // Sum the distance, load, serviceDuration and time associated
            // with the vehicle traveling from the depot to the next client
            distance += params->timeCost.get(route[idx - 1], route[idx]);
            load += params->cli[route[idx]].demand;

            time += params->cli[route[idx - 1]].serviceDuration
                    + params->timeCost.get(route[idx - 1], route[idx]);

            // Add possible waiting time
            if (time < params->cli[route[idx]].earliestArrival)
            {
                waitTime += params->cli[route[idx]].earliestArrival - time;
                time = params->cli[route[idx]].earliestArrival;
            }

            // Add possible time warp
            if (time > params->cli[route[idx]].latestArrival)
            {
                timeWarp += time - params->cli[route[idx]].latestArrival;
                time = params->cli[route[idx]].latestArrival;
            }

            // Update predecessors and successors
            predecessors[route[idx]] = route[idx - 1];
            successors[route[idx - 1]] = route[idx];
        }

        // For the last client, the successors is the depot. Also update the
        // distance and time
        successors[route.back()] = 0;
        distance += params->timeCost.get(route.back(), 0);
        time += params->cli[route.back()].serviceDuration
                + params->timeCost.get(route.back(), 0);

        // For the depot, we only need to check the end of the time window
        // (add possible time warp)
        timeWarp += std::max(time - params->cli[0].latestArrival, 0);

        // Whole solution stats
        costs.distance += distance;
        costs.waitTime += waitTime;
        costs.timeWarp += timeWarp;
        costs.capacityExcess += std::max(load - params->vehicleCapacity, 0);
    }

    // When all vehicles are dealt with, calculated total penalized cost and
    // check if the solution is feasible. (Wait time does not affect
    // feasibility)
    costs.penalizedCost = static_cast<double>(
        costs.distance + costs.capacityExcess * params->penaltyCapacity
        + costs.timeWarp * params->penaltyTimeWarp
        + costs.waitTime * params->penaltyWaitTime);
}

void Individual::removeProximity(Individual *other)
{
    auto it = indivsPerProximity.begin();
    while (it->second != other)
        ++it;

    indivsPerProximity.erase(it);
}

double Individual::brokenPairsDistance(Individual *other)
{
    int diffs = 0;

    for (int j = 1; j <= params->nbClients; j++)
    {
        // Increase the difference if the successor of j in this individual is
        // not directly linked to j in other
        diffs += successors[j] != other->successors[j]
                 && successors[j] != other->predecessors[j];

        // Increase the difference if the predecessor of j in this individual is
        // not directly linked to j in other
        diffs += predecessors[j] == 0 && other->predecessors[j] != 0
                 && other->successors[j] != 0;
    }

    return static_cast<double>(diffs) / params->nbClients;
}

double Individual::avgBrokenPairsDistanceClosest(size_t nbClosest) const
{
    double result = 0;
    size_t maxSize = std::min(nbClosest, indivsPerProximity.size());
    auto it = indivsPerProximity.begin();

    for (size_t itemCount = 0; itemCount != maxSize; ++itemCount)
    {
        result += it->first;
        ++it;
    }

    return result / static_cast<double>(maxSize);
}

void Individual::exportCVRPLibFormat(std::string const &path) const
{
    std::ofstream out(path);

    if (!out)
        throw std::runtime_error("Could not open " + path);

    for (size_t rIdx = 0; rIdx != costs.nbRoutes; ++rIdx)
    {
        out << "Route #" << rIdx + 1 << ":";  // route number

        for (int cIdx : routeChrom[rIdx])
            out << " " << cIdx;  // client index

        out << '\n';
    }

    out << "Cost " << costs.penalizedCost << '\n';
    out << "Time " << params->getElapsedTime() << '\n';
}

bool Individual::operator==(Individual const &other) const
{
    auto diff = std::abs(costs.penalizedCost - other.costs.penalizedCost);
    return diff < MY_EPSILON && tourChrom == other.tourChrom
           && routeChrom == other.routeChrom;
}

Individual::Individual(Params *params, bool initAndShuffle)
    : params(params),
      successors(params->nbClients + 1),
      predecessors(params->nbClients + 1),
      biasedFitness(0),
      tourChrom(params->nbClients),
      routeChrom(params->nbVehicles)
{
    if (initAndShuffle)
    {
        std::iota(tourChrom.begin(), tourChrom.end(), 1);
        std::shuffle(tourChrom.begin(), tourChrom.end(), params->rng);

        makeRoutes();
    }
}

Individual::Individual() : params(nullptr), biasedFitness(0)
{
    costs.penalizedCost = 1.e30;
}
