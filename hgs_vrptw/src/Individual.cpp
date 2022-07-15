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
    // TODO clean this up
    int maxVehicles = params->nbVehicles;

    auto cliSplit = std::vector<ClientSplit>(params->nbClients + 1);

    // potential[0][t] is the shortest path cost from 0 to t
    auto potential = std::vector<std::vector<double>>(
        params->nbVehicles + 1,
        std::vector<double>(params->nbClients + 1, 1.e30));

    // The next variable pred stores the client starting the route of a given
    // client. So pred[k] is the client starting the route where k is also in.
    // Index of the predecessor in an optimal path
    auto pred = std::vector<std::vector<int>>(
        params->nbVehicles + 1, std::vector<int>(params->nbClients + 1, 0));

    // Cumulative distance. sumDistance[i] for i > 1 contains the sum of
    // distances : sum_{k=1}^{i-1} d_{k,k+1}
    auto sumDistance = std::vector<int>(params->nbClients + 1, 0);

    // Cumulative demand. sumLoad[i] for i >= 1 contains the sum of loads :
    // sum_{k=1}^{i} q_k
    auto sumLoad = std::vector<int>(params->nbClients + 1, 0);

    // Cumulative service time. sumService[i] for i >= 1 contains the sum of
    // service time : sum_{k=1}^{i} s_k
    auto sumService = std::vector<int>(params->nbClients + 1, 0);

    // To be called with i < j only
    // Computes the cost of propagating the label i until j
    auto propagate = [&](int i, int j, int k) {
        assert(i < j);
        return potential[k][i] + sumDistance[j] - sumDistance[i + 1]
               + cliSplit[i + 1].d0_x + cliSplit[j].dx_0
               + params->penaltyCapacity
                     * std::max(
                         sumLoad[j] - sumLoad[i] - params->vehicleCapacity, 0);
    };

    // Tests if i dominates j as a predecessor for all nodes x >= j+1
    // We assume that i < j
    auto dominates = [&](int i, int j, int k) {
        assert(i < j);
        return potential[k][j] + cliSplit[j + 1].d0_x
               > potential[k][i] + cliSplit[i + 1].d0_x + sumDistance[j + 1]
                     - sumDistance[i + 1]
                     + params->penaltyCapacity * (sumLoad[j] - sumLoad[i]);
    };

    // Tests if j dominates i as a predecessor for all nodes x >= j+1
    // We assume that i < j
    auto dominatesRight = [&](int i, int j, int k) {
        assert(i < j);
        return potential[k][j] + cliSplit[j + 1].d0_x
               < potential[k][i] + cliSplit[i + 1].d0_x + sumDistance[j + 1]
                     - sumDistance[i + 1] + MY_EPSILON;
    };

    // Loop over all clients, excluding the depot
    for (int idx = 1; idx <= params->nbClients; idx++)
    {
        auto const curr = tourChrom[idx - 1];
        auto const next = tourChrom[idx];

        cliSplit[idx].demand = params->cli[curr].demand;
        cliSplit[idx].serviceTime = params->cli[curr].serviceDuration;
        cliSplit[idx].d0_x = params->timeCost.get(0, curr);
        cliSplit[idx].dx_0 = params->timeCost.get(curr, 0);

        // The distance to the next client is INT_MIN for the last client
        auto const dist = params->timeCost.get(curr, next);
        cliSplit[idx].dnext = idx < params->nbClients ? dist : INT_MIN;

        // Store cumulative data on the demand, service time, and distance
        sumLoad[idx] = sumLoad[idx - 1] + cliSplit[idx].demand;
        sumService[idx] = sumService[idx - 1] + cliSplit[idx].serviceTime;
        sumDistance[idx] = sumDistance[idx - 1] + cliSplit[idx - 1].dnext;
    }

    // Reinitialize the potential structure
    std::fill(potential[0].begin(), potential[0].end(), 1.e30);

    auto deq = std::deque<int>(params->nbClients + 1);
    deq.push_front(0);  // depot

    // Loop over all clients, excluding the depot
    for (int i = 1; i <= params->nbClients; i++)
    {
        // The front (which is the depot in the first loop) is the best
        // predecessor for i
        potential[0][i] = propagate(deq.front(), i, 0);
        pred[0][i] = deq.front();

        // Check if i is not the last client
        if (i < params->nbClients)
        {
            // If i is not dominated by the last of the pile
            if (!dominates(deq.back(), i, 0))
            {
                // Then i will be inserted, need to remove whoever is
                // dominated by i
                while (!deq.empty() && dominatesRight(deq.back(), i, 0))
                    deq.pop_back();

                deq.push_back(i);
            }

            // Check iteratively if front is dominated by the next front
            while (deq.size() > 1
                   && propagate(deq.front(), i + 1, 0)
                          > propagate(deq[1], i + 1, 0) - MY_EPSILON)
            {
                deq.pop_front();
            }
        }
    }

    // Check if the cost of the last client is still very large. In that case,
    // the Split algorithm did not reach the last client
    if (potential[0][params->nbClients] > 1.e29)
        throw std::string("No split has been propagated to the last node");

    // Loop over all vehicles, clear the route, get the predecessor and create a
    // new routeChrom for that route
    int end = params->nbClients;
    for (int k = maxVehicles - 1; k >= 0; k--)
    {
        routeChrom[k].clear();  // clear route and re-insert nodes

        // Loop from the begin to the end of the route corresponding to this
        // vehicle
        int begin = pred[0][end];
        for (int ii = begin; ii < end; ii++)
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
