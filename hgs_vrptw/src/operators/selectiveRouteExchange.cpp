#include "operators/crossover.h"

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <algorithm>
#include <unordered_set>

namespace
{
void insertUnplannedTasks(Params const &params,
                          std::vector<std::vector<int>> &routes,
                          std::unordered_set<int> &unplannedTasks)
{
    // Initialize some variables
    int newDistanceToInsert = INT_MAX;    // TODO:
    int newDistanceFromInsert = INT_MAX;  // TODO:
    int distanceDelta = INT_MAX;          // TODO:

    // Loop over all unplannedTasks
    for (int c : unplannedTasks)
    {
        // Get the earliest and latest possible arrival at the client
        int earliestArrival = params.clients[c].twEarly;
        int latestArrival = params.clients[c].twLate;

        int bestDistance = INT_MAX;
        std::pair<int, int> bestLocation;

        // Loop over all routes
        for (int r = 0; r < params.nbVehicles; r++)
        {
            // Go to the next route if this route is empty
            if (routes[r].empty())
            {
                continue;
            }

            newDistanceFromInsert = params.dist(c, routes[r][0]);
            if (earliestArrival + newDistanceFromInsert
                < params.clients[routes[r][0]].twLate)
            {
                distanceDelta = params.dist(0, c) + newDistanceToInsert
                                - params.dist(0, routes[r][0]);
                if (distanceDelta < bestDistance)
                {
                    bestDistance = distanceDelta;
                    bestLocation = {r, 0};
                }
            }

            for (int i = 1; i < static_cast<int>(routes[r].size()); i++)
            {
                newDistanceToInsert = params.dist(routes[r][i - 1], c);
                newDistanceFromInsert = params.dist(c, routes[r][i]);
                if (params.clients[routes[r][i - 1]].twEarly
                            + newDistanceToInsert
                        < latestArrival
                    && earliestArrival + newDistanceFromInsert
                           < params.clients[routes[r][i]].twLate)
                {
                    distanceDelta
                        = newDistanceToInsert + newDistanceFromInsert
                          - params.dist(routes[r][i - 1], routes[r][i]);
                    if (distanceDelta < bestDistance)
                    {
                        bestDistance = distanceDelta;
                        bestLocation = {r, i};
                    }
                }
            }

            newDistanceToInsert = params.dist(routes[r].back(), c);
            if (params.clients[routes[r].back()].twEarly + newDistanceToInsert
                < latestArrival)
            {
                distanceDelta = newDistanceToInsert + params.dist(c, 0)
                                - params.dist(routes[r].back(), 0);
                if (distanceDelta < bestDistance)
                {
                    bestDistance = distanceDelta;
                    bestLocation = {r, static_cast<int>(routes[r].size())};
                }
            }
        }

        routes[bestLocation.first].insert(
            routes[bestLocation.first].begin() + bestLocation.second, c);
    }
}
}  // namespace

Individual selectiveRouteExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng)
{
    // Get the number of routes of both parents
    int nOfRoutesA = parents.first->numRoutes();
    int nOfRoutesB = parents.second->numRoutes();

    auto const &routesA = parents.first->getRoutes();
    auto const &routesB = parents.second->getRoutes();

    // Picking the start index of routes to replace of parent A
    // We like to replace routes with a large overlap of tasks, so we choose
    // adjacent routes (they are sorted on polar angle)
    int startA = rng.randint(nOfRoutesA);
    int nOfMovedRoutes
        = std::min(nOfRoutesA, nOfRoutesB) == 1  // prevent moving no routes
              ? 1
              : rng() % (std::min(nOfRoutesA - 1, nOfRoutesB - 1)) + 1;
    int startB = startA < nOfRoutesB ? startA : 0;

    std::unordered_set<int> clientsInSelectedA;
    for (int r = 0; r < nOfMovedRoutes; r++)
    {
        // Insert the first
        clientsInSelectedA.insert(routesA[(startA + r) % nOfRoutesA].begin(),
                                  routesA[(startA + r) % nOfRoutesA].end());
    }

    std::unordered_set<int> clientsInSelectedB;
    for (int r = 0; r < nOfMovedRoutes; r++)
    {
        clientsInSelectedB.insert(routesB[(startB + r) % nOfRoutesB].begin(),
                                  routesB[(startB + r) % nOfRoutesB].end());
    }

    bool improved = true;
    while (improved)
    {
        // Difference for moving 'left' in parent A
        const int differenceALeft
            = static_cast<int>(std::count_if(
                  routesA[(startA - 1 + nOfRoutesA) % nOfRoutesA].begin(),
                  routesA[(startA - 1 + nOfRoutesA) % nOfRoutesA].end(),
                  [&clientsInSelectedB](int c) {
                      return clientsInSelectedB.find(c)
                             == clientsInSelectedB.end();
                  }))
              - static_cast<int>(std::count_if(
                  routesA[(startA + nOfMovedRoutes - 1) % nOfRoutesA].begin(),
                  routesA[(startA + nOfMovedRoutes - 1) % nOfRoutesA].end(),
                  [&clientsInSelectedB](int c) {
                      return clientsInSelectedB.find(c)
                             == clientsInSelectedB.end();
                  }));

        // Difference for moving 'right' in parent A
        const int differenceARight
            = static_cast<int>(std::count_if(
                  routesA[(startA + nOfMovedRoutes) % nOfRoutesA].begin(),
                  routesA[(startA + nOfMovedRoutes) % nOfRoutesA].end(),
                  [&clientsInSelectedB](int c) {
                      return clientsInSelectedB.find(c)
                             == clientsInSelectedB.end();
                  }))
              - static_cast<int>(
                  std::count_if(routesA[startA].begin(),
                                routesA[startA].end(),
                                [&clientsInSelectedB](int c) {
                                    return clientsInSelectedB.find(c)
                                           == clientsInSelectedB.end();
                                }));

        // Difference for moving 'left' in parent B
        const int differenceBLeft
            = static_cast<int>(std::count_if(
                  routesB[(startB - 1 + nOfMovedRoutes) % nOfRoutesB].begin(),
                  routesB[(startB - 1 + nOfMovedRoutes) % nOfRoutesB].end(),
                  [&clientsInSelectedA](int c) {
                      return clientsInSelectedA.find(c)
                             != clientsInSelectedA.end();
                  }))
              - static_cast<int>(std::count_if(
                  routesB[(startB - 1 + nOfRoutesB) % nOfRoutesB].begin(),
                  routesB[(startB - 1 + nOfRoutesB) % nOfRoutesB].end(),
                  [&clientsInSelectedA](int c) {
                      return clientsInSelectedA.find(c)
                             != clientsInSelectedA.end();
                  }));

        // Difference for moving 'right' in parent B
        const int differenceBRight
            = static_cast<int>(
                  std::count_if(routesB[startB].begin(),
                                routesB[startB].end(),
                                [&clientsInSelectedA](int c) {
                                    return clientsInSelectedA.find(c)
                                           != clientsInSelectedA.end();
                                }))
              - static_cast<int>(std::count_if(
                  routesB[(startB + nOfMovedRoutes) % nOfRoutesB].begin(),
                  routesB[(startB + nOfMovedRoutes) % nOfRoutesB].end(),
                  [&clientsInSelectedA](int c) {
                      return clientsInSelectedA.find(c)
                             != clientsInSelectedA.end();
                  }));

        const int bestDifference = std::min({differenceALeft,
                                             differenceARight,
                                             differenceBLeft,
                                             differenceBRight});

        if (bestDifference < 0)
        {
            if (bestDifference == differenceALeft)
            {
                for (int c :
                     routesA[(startA + nOfMovedRoutes - 1) % nOfRoutesA])
                {
                    clientsInSelectedA.erase(clientsInSelectedA.find(c));
                }
                startA = (startA - 1 + nOfRoutesA) % nOfRoutesA;
                for (int c : routesA[startA])
                {
                    clientsInSelectedA.insert(c);
                }
            }
            else if (bestDifference == differenceARight)
            {
                for (int c : routesA[startA])
                {
                    clientsInSelectedA.erase(clientsInSelectedA.find(c));
                }
                startA = (startA + 1) % nOfRoutesA;
                for (int c :
                     routesA[(startA + nOfMovedRoutes - 1) % nOfRoutesA])
                {
                    clientsInSelectedA.insert(c);
                }
            }
            else if (bestDifference == differenceBLeft)
            {
                for (int c :
                     routesB[(startB + nOfMovedRoutes - 1) % nOfRoutesB])
                {
                    clientsInSelectedB.erase(clientsInSelectedB.find(c));
                }
                startB = (startB - 1 + nOfRoutesB) % nOfRoutesB;
                for (int c : routesB[startB])
                {
                    clientsInSelectedB.insert(c);
                }
            }
            else if (bestDifference == differenceBRight)
            {
                for (int c : routesB[startB])
                {
                    clientsInSelectedB.erase(clientsInSelectedB.find(c));
                }
                startB = (startB + 1) % nOfRoutesB;
                for (int c :
                     routesB[(startB + nOfMovedRoutes - 1) % nOfRoutesB])
                {
                    clientsInSelectedB.insert(c);
                }
            }
        }
        else
        {
            improved = false;
        }
    }

    // Identify differences between route sets
    std::unordered_set<int> clientsInSelectedANotB;
    std::copy_if(
        clientsInSelectedA.begin(),
        clientsInSelectedA.end(),
        std::inserter(clientsInSelectedANotB, clientsInSelectedANotB.end()),
        [&clientsInSelectedB](int c)
        { return clientsInSelectedB.find(c) == clientsInSelectedB.end(); });

    std::unordered_set<int> clientsInSelectedBNotA;
    std::copy_if(
        clientsInSelectedB.begin(),
        clientsInSelectedB.end(),
        std::inserter(clientsInSelectedBNotA, clientsInSelectedBNotA.end()),
        [&clientsInSelectedA](int c)
        { return clientsInSelectedA.find(c) == clientsInSelectedA.end(); });

    std::vector<std::vector<int>> routes1(params.nbVehicles);
    std::vector<std::vector<int>> routes2(params.nbVehicles);

    // Replace selected routes from parent A with routes from parent B
    for (int r = 0; r < nOfMovedRoutes; r++)
    {
        int indexA = (startA + r) % nOfRoutesA;
        int indexB = (startB + r) % nOfRoutesB;

        for (int c : routesB[indexB])
        {
            routes1[indexA].push_back(c);
            if (clientsInSelectedBNotA.find(c) == clientsInSelectedBNotA.end())
            {
                routes2[indexA].push_back(c);
            }
        }
    }

    // Move routes from parent A that are kept
    for (int r = nOfMovedRoutes; r < nOfRoutesA; r++)
    {
        int indexA = (startA + r) % nOfRoutesA;
        routes1[indexA].clear();
        routes2[indexA].clear();

        for (int c : routesA[indexA])
        {
            if (clientsInSelectedBNotA.find(c) == clientsInSelectedBNotA.end())
            {
                routes1[indexA].push_back(c);
            }
            routes2[indexA].push_back(c);
        }
    }

    // Delete any remaining routes that still lived in offspring
    for (int r = nOfRoutesA; r < params.nbVehicles; r++)
    {
        routes1[r].clear();
        routes2[r].clear();
    }

    // Step 3: Insert unplanned clients (those that were in the removed routes
    // of A but not the inserted routes of B)
    insertUnplannedTasks(params, routes1, clientsInSelectedANotB);
    insertUnplannedTasks(params, routes2, clientsInSelectedANotB);

    Individual indiv1{&params, parents.first->getTour(), routes1};
    Individual indiv2{&params, parents.second->getTour(), routes2};

    return indiv1.cost() < indiv2.cost() ? indiv1 : indiv2;
}
