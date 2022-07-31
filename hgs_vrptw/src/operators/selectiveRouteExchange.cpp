#include "operators/crossover.h"

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <algorithm>
#include <array>
#include <unordered_set>

namespace
{
using ClientSet = std::unordered_set<int>;
using Routes = std::vector<std::vector<int>>;

void addUnplannedToRoute(Params const &params,
                         Routes &routes,
                         ClientSet const &unplanned)
{
    int distToInsert = INT_MAX;  // TODO:

    // Loop over all unplanned
    for (int c : unplanned)
    {
        int twEarly = params.clients[c].twEarly;
        int twLate = params.clients[c].twLate;

        // Used as a tuple of (delta dist, route idx, offset idx) elements.
        std::array<int, 3> best = {INT_MAX, 0, 0};

        for (int r = 0; r < params.nbVehicles; r++)
        {
            if (routes[r].empty())
                continue;  // TODO can this be break?

            int size = static_cast<int>(routes[r].size());

            int distFromInsert = params.dist(c, routes[r][0]);
            if (twEarly + distFromInsert < params.clients[routes[r][0]].twLate)
            {
                int distanceDelta = params.dist(0, c) + distToInsert
                                    - params.dist(0, routes[r][0]);

                if (distanceDelta < best[0])
                    best = {distanceDelta, r, 0};
            }

            for (int i = 1; i < size; i++)
            {
                distToInsert = params.dist(routes[r][i - 1], c);
                distFromInsert = params.dist(c, routes[r][i]);
                if (params.clients[routes[r][i - 1]].twEarly + distToInsert
                        < twLate
                    && twEarly + distFromInsert
                           < params.clients[routes[r][i]].twLate)
                {
                    int distanceDelta
                        = distToInsert + distFromInsert
                          - params.dist(routes[r][i - 1], routes[r][i]);

                    if (distanceDelta < best[0])
                        best = {distanceDelta, r, i};
                }
            }

            distToInsert = params.dist(routes[r].back(), c);
            if (params.clients[routes[r].back()].twEarly + distToInsert
                < twLate)
            {
                int distanceDelta
                    = distToInsert + params.dist(c, 0) - params.dist(size, 0);

                if (distanceDelta < best[0])
                    best = {distanceDelta, r, size};
            }
        }

        routes[best[1]].insert(routes[best[1]].begin() + best[2], c);
    }
}
}  // namespace

Individual selectiveRouteExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng)
{
    // Get the number of routes of both parents
    size_t nOfRoutesA = parents.first->numRoutes();
    size_t nOfRoutesB = parents.second->numRoutes();

    auto const &routesA = parents.first->getRoutes();
    auto const &routesB = parents.second->getRoutes();

    // Picking the start index of routes to replace of parent A. We like to
    // replace routes with a large overlap of tasks, so we choose adjacent
    // routes (they are sorted on polar angle)
    size_t startA = rng.randint(static_cast<int>(nOfRoutesA));
    size_t nOfMovedRoutes
        = std::min(nOfRoutesA, nOfRoutesB) == 1  // prevent moving no routes
              ? 1
              : rng() % (std::min(nOfRoutesA - 1, nOfRoutesB - 1)) + 1;
    size_t startB = startA < nOfRoutesB ? startA : 0;

    ClientSet clientsInSelectedA;
    for (size_t r = 0; r < nOfMovedRoutes; r++)
        clientsInSelectedA.insert(routesA[(startA + r) % nOfRoutesA].begin(),
                                  routesA[(startA + r) % nOfRoutesA].end());

    ClientSet clientsInSelectedB;
    for (size_t r = 0; r < nOfMovedRoutes; r++)
        clientsInSelectedB.insert(routesB[(startB + r) % nOfRoutesB].begin(),
                                  routesB[(startB + r) % nOfRoutesB].end());

    while (true)
    {
        // Difference for moving 'left' in parent A
        int const differenceALeft
            = static_cast<int>(std::count_if(
                  routesA[(startA - 1 + nOfRoutesA) % nOfRoutesA].begin(),
                  routesA[(startA - 1 + nOfRoutesA) % nOfRoutesA].end(),
                  [&clientsInSelectedB](int c)
                  { return !clientsInSelectedB.contains(c); }))
              - static_cast<int>(std::count_if(
                  routesA[(startA + nOfMovedRoutes - 1) % nOfRoutesA].begin(),
                  routesA[(startA + nOfMovedRoutes - 1) % nOfRoutesA].end(),
                  [&clientsInSelectedB](int c)
                  { return !clientsInSelectedB.contains(c); }));

        // Difference for moving 'right' in parent A
        int const differenceARight
            = static_cast<int>(std::count_if(
                  routesA[(startA + nOfMovedRoutes) % nOfRoutesA].begin(),
                  routesA[(startA + nOfMovedRoutes) % nOfRoutesA].end(),
                  [&clientsInSelectedB](int c)
                  { return !clientsInSelectedB.contains(c); }))
              - static_cast<int>(
                  std::count_if(routesA[startA].begin(),
                                routesA[startA].end(),
                                [&clientsInSelectedB](int c)
                                { return !clientsInSelectedB.contains(c); }));

        // Difference for moving 'left' in parent B
        int const differenceBLeft
            = static_cast<int>(std::count_if(
                  routesB[(startB - 1 + nOfMovedRoutes) % nOfRoutesB].begin(),
                  routesB[(startB - 1 + nOfMovedRoutes) % nOfRoutesB].end(),
                  [&clientsInSelectedA](int c)
                  { return clientsInSelectedA.contains(c); }))
              - static_cast<int>(std::count_if(
                  routesB[(startB - 1 + nOfRoutesB) % nOfRoutesB].begin(),
                  routesB[(startB - 1 + nOfRoutesB) % nOfRoutesB].end(),
                  [&clientsInSelectedA](int c)
                  { return clientsInSelectedA.contains(c); }));

        // Difference for moving 'right' in parent B
        int const differenceBRight
            = static_cast<int>(
                  std::count_if(routesB[startB].begin(),
                                routesB[startB].end(),
                                [&clientsInSelectedA](int c)
                                { return clientsInSelectedA.contains(c); }))
              - static_cast<int>(std::count_if(
                  routesB[(startB + nOfMovedRoutes) % nOfRoutesB].begin(),
                  routesB[(startB + nOfMovedRoutes) % nOfRoutesB].end(),
                  [&clientsInSelectedA](int c)
                  { return clientsInSelectedA.contains(c); }));

        int const bestDifference = std::min({differenceALeft,
                                             differenceARight,
                                             differenceBLeft,
                                             differenceBRight});

        if (bestDifference >= 0)  // there are no further improving moves
            break;

        if (bestDifference == differenceALeft)
        {
            for (int c : routesA[(startA + nOfMovedRoutes - 1) % nOfRoutesA])
                clientsInSelectedA.erase(clientsInSelectedA.find(c));

            startA = (startA - 1 + nOfRoutesA) % nOfRoutesA;

            for (int c : routesA[startA])
                clientsInSelectedA.insert(c);
        }
        else if (bestDifference == differenceARight)
        {
            for (int c : routesA[startA])
                clientsInSelectedA.erase(clientsInSelectedA.find(c));

            startA = (startA + 1) % nOfRoutesA;

            for (int c : routesA[(startA + nOfMovedRoutes - 1) % nOfRoutesA])
                clientsInSelectedA.insert(c);
        }
        else if (bestDifference == differenceBLeft)
        {
            for (int c : routesB[(startB + nOfMovedRoutes - 1) % nOfRoutesB])
                clientsInSelectedB.erase(clientsInSelectedB.find(c));

            startB = (startB - 1 + nOfRoutesB) % nOfRoutesB;

            for (int c : routesB[startB])
                clientsInSelectedB.insert(c);
        }
        else if (bestDifference == differenceBRight)
        {
            for (int c : routesB[startB])
                clientsInSelectedB.erase(clientsInSelectedB.find(c));

            startB = (startB + 1) % nOfRoutesB;
            for (int c : routesB[(startB + nOfMovedRoutes - 1) % nOfRoutesB])
                clientsInSelectedB.insert(c);
        }
    }

    // Identify differences between route sets
    ClientSet clientsInSelectedANotB;
    std::copy_if(
        clientsInSelectedA.begin(),
        clientsInSelectedA.end(),
        std::inserter(clientsInSelectedANotB, clientsInSelectedANotB.end()),
        [&clientsInSelectedB](int c)
        { return clientsInSelectedB.find(c) == clientsInSelectedB.end(); });

    ClientSet clientsInSelectedBNotA;
    std::copy_if(
        clientsInSelectedB.begin(),
        clientsInSelectedB.end(),
        std::inserter(clientsInSelectedBNotA, clientsInSelectedBNotA.end()),
        [&clientsInSelectedA](int c)
        { return clientsInSelectedA.find(c) == clientsInSelectedA.end(); });

    Routes routes1(params.nbVehicles);
    Routes routes2(params.nbVehicles);

    // Replace selected routes from parent A with routes from parent B
    for (size_t r = 0; r < nOfMovedRoutes; r++)
    {
        size_t indexA = (startA + r) % nOfRoutesA;
        size_t indexB = (startB + r) % nOfRoutesB;

        for (int c : routesB[indexB])
        {
            routes1[indexA].push_back(c);

            if (clientsInSelectedBNotA.find(c) == clientsInSelectedBNotA.end())
                routes2[indexA].push_back(c);
        }
    }

    // Move routes from parent A that are kept
    for (size_t r = nOfMovedRoutes; r < nOfRoutesA; r++)
    {
        size_t indexA = (startA + r) % nOfRoutesA;
        routes1[indexA].clear();
        routes2[indexA].clear();

        for (int c : routesA[indexA])
        {
            if (clientsInSelectedBNotA.find(c) == clientsInSelectedBNotA.end())
                routes1[indexA].push_back(c);

            routes2[indexA].push_back(c);
        }
    }

    // Delete any remaining routes that still lived in offspring
    for (size_t r = nOfRoutesA; r < static_cast<size_t>(params.nbVehicles); r++)
    {
        routes1[r].clear();
        routes2[r].clear();
    }

    // Step 3: Insert unplanned clients (those that were in the removed routes
    // of A, but not the inserted routes of B)
    addUnplannedToRoute(params, routes1, clientsInSelectedANotB);
    addUnplannedToRoute(params, routes2, clientsInSelectedANotB);

    Individual indiv1{&params, parents.first->getTour(), routes1};
    Individual indiv2{&params, parents.second->getTour(), routes2};

    return indiv1.cost() < indiv2.cost() ? indiv1 : indiv2;
}
