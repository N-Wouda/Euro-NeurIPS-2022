#include "Individual.h"

#include "Params.h"

#include <algorithm>
#include <cassert>
#include <deque>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

void Individual::makeRoutes()
{
    // TODO clean this up further

    auto cliSplit = std::vector<ClientSplit>(params->nbClients + 1);
    auto preds = std::vector<int>(params->nbClients + 1, 0);  // predecessors
    auto pathCosts = std::vector<double>(params->nbClients + 1, 1.e30);
    pathCosts[0] = 0;

    auto cumDist = std::vector<int>(params->nbClients + 1, 0);
    auto cumLoad = std::vector<int>(params->nbClients + 1, 0);
    auto cumServ = std::vector<int>(params->nbClients + 1, 0);

    // Computes the cost of propagating label i to j
    auto propagate = [&](int i, int j)
    {
        assert(i < j);
        auto const excessCapacity
            = std::max(cumLoad[j] - cumLoad[i] - params->vehicleCapacity, 0);

        return pathCosts[i] + cumDist[j] - cumDist[i + 1] + cliSplit[i + 1].d0_x
               + cliSplit[j].dx_0 + params->penaltyCapacity * excessCapacity;
    };

    // Tests if i dominates j as a predecessor for all nodes x >= j+1
    auto dominates = [&](int i, int j)
    {
        auto const lhs = pathCosts[j] + cliSplit[j + 1].d0_x;
        auto const rhs = pathCosts[i] + cliSplit[i + 1].d0_x + cumDist[j + 1]
                         - cumDist[i + 1]
                         + params->penaltyCapacity * (cumLoad[j] - cumLoad[i]);

        return lhs + MY_EPSILON > rhs;
    };

    // Loop over all clients, excluding the depot
    for (int idx = 1; idx <= params->nbClients; idx++)
    {
        auto const curr = tourChrom[idx - 1];
        auto const next = tourChrom[idx];
        auto const dist = idx < params->nbClients  // INT_MIN for last client
                              ? params->timeCost.get(curr, next)
                              : INT_MIN;

        cliSplit[idx] = {params->cli[curr].demand,
                         params->cli[curr].serviceDuration,
                         params->timeCost.get(0, curr),
                         params->timeCost.get(curr, 0),
                         dist};

        cumLoad[idx] = cumLoad[idx - 1] + cliSplit[idx].demand;
        cumServ[idx] = cumServ[idx - 1] + cliSplit[idx].serviceTime;
        cumDist[idx] = cumDist[idx - 1] + cliSplit[idx - 1].dnext;
    }

    auto deq = std::deque<int>(params->nbClients + 1);
    deq.push_front(0);  // depot

    for (int i = 1; i <= params->nbClients; i++)  // exclude depot
    {
        // The front (which is the depot in the first loop) is the best
        // predecessor for i
        pathCosts[i] = propagate(deq.front(), i);
        preds[i] = deq.front();

        // Check if i is not the last client
        if (i < params->nbClients)
        {
            // If i is not dominated by the last of the pile
            if (!dominates(deq.back(), i))
            {
                // Then i will be inserted, need to remove whoever is
                // dominated by i
                while (!deq.empty() && dominates(i, deq.back()))
                    deq.pop_back();

                deq.push_back(i);
            }

            // Check iteratively if front is dominated by the next front
            while (deq.size() > 1
                   && propagate(deq.front(), i + 1)
                          > propagate(deq[1], i + 1) - MY_EPSILON)
            {
                deq.pop_front();
            }
        }
    }

    if (pathCosts[params->nbClients] > 1.e29)  // has not been updated
        throw std::runtime_error("No split solution reached the last client");

    // Loop over all vehicles, clear the route, get the predecessor and create a
    // new routeChrom for that route
    int end = params->nbClients;
    for (int k = params->nbVehicles - 1; k >= 0; k--)
    {
        int begin = preds[end];

        routeChrom[k].clear();

        for (int ii = begin; ii < end; ii++)  // TODO use slice
            routeChrom[k].push_back(tourChrom[ii]);

        end = begin;
    }

    evaluateCompleteCost();
}

void Individual::evaluateCompleteCost()
{
    // Create an object to store all information regarding solution costs
    costs = CostSol();
    // Loop over all routes that are not empty
    for (int r = 0; r < params->nbVehicles; r++)
    {
        if (!routeChrom[r].empty())
        {
            int latestReleaseTime = params->cli[routeChrom[r][0]].releaseTime;
            for (int i = 1; i < static_cast<int>(routeChrom[r].size()); i++)
            {
                latestReleaseTime
                    = std::max(latestReleaseTime,
                               params->cli[routeChrom[r][i]].releaseTime);
            }
            // Get the distance, load, serviceDuration and time associated with
            // the vehicle traveling from the depot to the first client Assume
            // depot has service time 0 and earliestArrival 0
            int distance = params->timeCost.get(0, routeChrom[r][0]);
            int load = params->cli[routeChrom[r][0]].demand;

            // Running time excludes service of current node. This is the time
            // that runs with the vehicle traveling We start the route at the
            // latest release time (or later, but then we can just wait and
            // there is no penalty for waiting)
            int time = latestReleaseTime + distance;
            int waitTime = 0;
            int timeWarp = 0;
            // Add possible waiting time
            if (time < params->cli[routeChrom[r][0]].earliestArrival)
            {
                // Don't add wait time since we can start route later
                // (doesn't really matter since there is no penalty anyway)
                // waitTime += params->cli[routeChrom[r][0]].earliestArrival -
                // time;
                time = params->cli[routeChrom[r][0]].earliestArrival;
            }
            // Add possible time warp
            else if (time > params->cli[routeChrom[r][0]].latestArrival)
            {
                timeWarp += time - params->cli[routeChrom[r][0]].latestArrival;
                time = params->cli[routeChrom[r][0]].latestArrival;
            }
            predecessors[routeChrom[r][0]] = 0;

            // Loop over all clients for this vehicle
            for (int i = 1; i < static_cast<int>(routeChrom[r].size()); i++)
            {
                // Sum the distance, load, serviceDuration and time associated
                // with the vehicle traveling from the depot to the next client
                distance += params->timeCost.get(routeChrom[r][i - 1],
                                                 routeChrom[r][i]);
                load += params->cli[routeChrom[r][i]].demand;

                time = time + params->cli[routeChrom[r][i - 1]].serviceDuration
                       + params->timeCost.get(routeChrom[r][i - 1],
                                              routeChrom[r][i]);

                // Add possible waiting time
                if (time < params->cli[routeChrom[r][i]].earliestArrival)
                {
                    waitTime
                        += params->cli[routeChrom[r][i]].earliestArrival - time;
                    time = params->cli[routeChrom[r][i]].earliestArrival;
                }
                // Add possible time warp
                else if (time > params->cli[routeChrom[r][i]].latestArrival)
                {
                    timeWarp
                        += time - params->cli[routeChrom[r][i]].latestArrival;
                    time = params->cli[routeChrom[r][i]].latestArrival;
                }

                // Update predecessors and successors
                predecessors[routeChrom[r][i]] = routeChrom[r][i - 1];
                successors[routeChrom[r][i - 1]] = routeChrom[r][i];
            }

            // For the last client, the successors is the depot. Also update the
            // distance and time
            successors[routeChrom[r][routeChrom[r].size() - 1]] = 0;
            distance += params->timeCost.get(
                routeChrom[r][routeChrom[r].size() - 1], 0);
            time = time
                   + params->cli[routeChrom[r][routeChrom[r].size() - 1]]
                         .serviceDuration
                   + params->timeCost.get(
                       routeChrom[r][routeChrom[r].size() - 1], 0);

            // For the depot, we only need to check the end of the time window
            // (add possible time warp)
            if (time > params->cli[0].latestArrival)
                timeWarp += time - params->cli[0].latestArrival;

            // Update variables that track stats on the whole solution (all
            // vehicles combined)
            costs.distance += distance;
            costs.waitTime += waitTime;
            costs.timeWarp += timeWarp;
            costs.nbRoutes++;

            if (load > params->vehicleCapacity)
            {
                costs.capacityExcess += load - params->vehicleCapacity;
            }
        }
    }

    // When all vehicles are dealt with, calculated total penalized cost and
    // check if the solution is feasible. (Wait time does not affect
    // feasibility)
    costs.penalizedCost = costs.distance
                          + costs.capacityExcess * params->penaltyCapacity
                          + costs.timeWarp * params->penaltyTimeWarp
                          + costs.waitTime * params->penaltyWaitTime;
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

    for (int k = 0; k < params->nbVehicles; k++)
        if (!routeChrom[k].empty())
        {
            // Route IDs start at 1 in the file format
            out << "Route #" << k + 1 << ":";

            for (int i : routeChrom[k])
                out << " " << i;

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

Individual::Individual(Params *params, bool initializeChromTAndShuffle)
    : params(params), biasedFitness(0)
{
    successors = std::vector<int>(params->nbClients + 1);
    predecessors = std::vector<int>(params->nbClients + 1);
    routeChrom = std::vector<std::vector<int>>(params->nbVehicles);
    tourChrom = std::vector<int>(params->nbClients);

    if (initializeChromTAndShuffle)
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
