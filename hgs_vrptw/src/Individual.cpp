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
    int capPenalty;

    std::vector<ClientSplit> splits;
    std::vector<int> predecessors;
    std::vector<int> pathCosts;

    std::vector<int> cumDist;
    std::vector<int> cumLoad;
    std::vector<int> cumServ;

    explicit ClientSplits(Params const &params, std::vector<int> const &tour)
        : vehicleCap(params.vehicleCapacity),
          capPenalty(params.penaltyCapacity),
          splits(params.nbClients + 1),
          predecessors(params.nbClients + 1, 0),
          pathCosts(params.nbClients + 1, INT_MAX),
          cumDist(params.nbClients + 1, 0),
          cumLoad(params.nbClients + 1, 0),
          cumServ(params.nbClients + 1, 0)
    {
        pathCosts[0] = 0;

        for (int idx = 1; idx <= params.nbClients; idx++)  // exclude depot
        {
            auto const curr = tour[idx - 1];
            auto const dist = idx < params.nbClients  // INT_MIN for last client
                                  ? params.dist(curr, tour[idx])
                                  : INT_MIN;

            splits[idx] = {params.clients[curr].demand,
                           params.clients[curr].servDur,
                           params.dist(0, curr),
                           params.dist(curr, 0),
                           dist};

            cumLoad[idx] = cumLoad[idx - 1] + splits[idx].demand;
            cumServ[idx] = cumServ[idx - 1] + splits[idx].serviceTime;
            cumDist[idx] = cumDist[idx - 1] + splits[idx - 1].dnext;
        }
    }

    // Computes the cost of propagating label i to j
    [[nodiscard]] inline int propagate(int i, int j) const
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

        return lhs >= rhs;
    }

    // Tests if j dominates i as a predecessor for all nodes x >= j + 1
    [[nodiscard]] inline bool rightDominates(int i, int j) const
    {
        assert(i < j);
        auto const lhs = pathCosts[j] + splits[j + 1].d0_x;
        auto const rhs = pathCosts[i] + splits[i + 1].d0_x + cumDist[j + 1]
                         - cumDist[i + 1];

        return lhs <= rhs;
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

            if (firstProp >= secondProp)
                deq.pop_front();
            else
                break;
        }
    }

    if (splits.pathCosts[params->nbClients] == INT_MAX)  // has not been updated
        throw std::runtime_error("No split solution reached the last client");

    int end = params->nbClients;
    for (auto &route : routeChrom)
    {
        route.clear();

        if (end != 0)  // assign routes
        {
            int begin = splits.predecessors[end];
            route = {tourChrom.begin() + begin, tourChrom.begin() + end};
            end = begin;
        }
    }

    evaluateCompleteCost();
}

void Individual::evaluateCompleteCost()
{
    // Reset fields before evaluating them again below.
    nbRoutes = 0;
    distance = 0;
    capacityExcess = 0;
    timeWarp = 0;

    for (auto const &route : routeChrom)
    {
        if (route.empty())  // First empty route. Due to makeRoutes() all
            break;          // subsequent routes are empty as well

        nbRoutes++;

        int lastRelease = 0;
        for (auto const idx : route)
            lastRelease
                = std::max(lastRelease, params->clients[idx].releaseTime);

        int rDist = params->dist(0, route[0]);
        int rTimeWarp = 0;

        int load = params->clients[route[0]].demand;
        int time = lastRelease + rDist;

        if (time < params->clients[route[0]].twEarly)
            time = params->clients[route[0]].twEarly;

        if (time > params->clients[route[0]].twLate)
        {
            rTimeWarp += time - params->clients[route[0]].twLate;
            time = params->clients[route[0]].twLate;
        }

        for (size_t idx = 1; idx < route.size(); idx++)
        {
            // Sum the rDist, load, servDur and time associated with the vehicle
            // traveling from the depot to the next client
            rDist += params->dist(route[idx - 1], route[idx]);
            load += params->clients[route[idx]].demand;

            time += params->clients[route[idx - 1]].servDur
                    + params->dist(route[idx - 1], route[idx]);

            // Add possible waiting time
            if (time < params->clients[route[idx]].twEarly)
                time = params->clients[route[idx]].twEarly;

            // Add possible time warp
            if (time > params->clients[route[idx]].twLate)
            {
                rTimeWarp += time - params->clients[route[idx]].twLate;
                time = params->clients[route[idx]].twLate;
            }
        }

        // For the last client, the successors is the depot. Also update the
        // rDist and time
        rDist += params->dist(route.back(), 0);
        time += params->clients[route.back()].servDur
                + params->dist(route.back(), 0);

        // For the depot, we only need to check the end of the time window
        // (add possible time warp)
        rTimeWarp += std::max(time - params->clients[0].twLate, 0);

        // Whole solution stats
        distance += rDist;
        timeWarp += rTimeWarp;
        capacityExcess += std::max(load - params->vehicleCapacity, 0);
    }
}

void Individual::brokenPairsDistance(Individual *other)
{
    auto const tNeighbours = this->getNeighbours();
    auto const oNeighbours = other->getNeighbours();

    int diffs = 0;

    for (int j = 1; j <= params->nbClients; j++)
    {
        auto const &[tPred, tSucc] = tNeighbours[j];
        auto const &[oPred, oSucc] = oNeighbours[j];

        // Increase the difference if the successor of j in this individual is
        // not directly linked to j in other
        diffs += tSucc != oSucc && tSucc != oPred;

        // Increase the difference if the predecessor of j in this individual is
        // not directly linked to j in other
        diffs += tPred == 0 && oPred != 0 && oSucc != 0;
    }

    double const dist = static_cast<double>(diffs) / params->nbClients;

    other->indivsPerProximity.insert({dist, this});
    indivsPerProximity.insert({dist, other});
}

double Individual::avgBrokenPairsDistanceClosest() const
{
    size_t maxSize
        = std::min(params->config.nbClose, indivsPerProximity.size());

    double result = 0;
    auto it = indivsPerProximity.begin();

    for (size_t itemCount = 0; itemCount != maxSize; ++itemCount)
    {
        result += it->first;
        ++it;
    }

    return result / static_cast<double>(maxSize);
}

void Individual::exportCVRPLibFormat(std::string const &path, double time) const
{
    std::ofstream out(path);

    if (!out)
        throw std::runtime_error("Could not open " + path);

    for (size_t rIdx = 0; rIdx != nbRoutes; ++rIdx)
    {
        out << "Route #" << rIdx + 1 << ":";  // route number
        for (int cIdx : routeChrom[rIdx])
            out << " " << cIdx;  // client index
        out << '\n';
    }

    out << "Cost " << cost() << '\n';
    out << "Time " << time << '\n';
}

std::vector<std::pair<int, int>> Individual::getNeighbours() const
{
    std::vector<std::pair<int, int>> neighbours(params->nbClients + 1);
    neighbours[0] = {0, 0};  // note that depot neighbours have no meaning

    for (auto const &route : routeChrom)
        for (size_t idx = 0; idx != route.size(); ++idx)
            neighbours[route[idx]]
                = {idx == 0 ? 0 : route[idx - 1],                  // pred
                   idx == route.size() - 1 ? 0 : route[idx + 1]};  // succ

    return neighbours;
}

Individual::Individual(Params *params, XorShift128 *rng)
    : params(params),
      tourChrom(params->nbClients),
      routeChrom(params->nbVehicles)
{
    std::iota(tourChrom.begin(), tourChrom.end(), 1);
    std::shuffle(tourChrom.begin(), tourChrom.end(), *rng);

    makeRoutes();
}

Individual::Individual(Params *params, Tour tour)
    : params(params), tourChrom(std::move(tour)), routeChrom(params->nbVehicles)
{
    makeRoutes();
}

Individual::Individual(Params *params, Tour tour, Routes routes)
    : params(params), tourChrom(std::move(tour)), routeChrom(std::move(routes))
{
    evaluateCompleteCost();
}

Individual::~Individual()
{
    for (auto &[_, other] : indivsPerProximity)  // remove ourselves from
    {                                            // other's proximity list
        auto it = other->indivsPerProximity.begin();
        while (it->second != this)
            ++it;

        other->indivsPerProximity.erase(it);
    }
}
