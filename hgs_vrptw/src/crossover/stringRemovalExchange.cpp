#include "crossover.h"

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <string>
#include <unordered_set>

namespace
{
using Parents = std::pair<Individual const *, Individual const *>;
using Client = int;
using ClientSet = std::unordered_set<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;
using Destroyed = std::pair<Routes, ClientSet>;

struct InsertPos  // best insert position, used to plan unplanned clients
{
    int deltaCost;
    Route *route;
    size_t offset;
};

void printRouteSize(Routes const routes, std::string text)
{
    // std::cout << text;
    // int tot = 0;
    // for (auto &route : routes)
    //     tot += route.size();
    // printf("%d\n", tot);
}

void printClientSet(ClientSet const route, std::string text)
{
    // std::cout << text;
    // for (auto c : route)
    //     printf("%d-", c);
    // printf("\n");
}

void printRoute(Route const route, std::string text)
{
    // std::cout << text;
    // for (auto c : route)
    //     printf("%d-", c);
    // printf("\n");
}

// Returns the indices of a random (sub)string with length card that
// contains the client
std::vector<int>
selectString(Route route, Client client, int card, XorShift128 &rng)
{
    std::vector<int>::iterator itr
        = std::find(route.begin(), route.end(), client);
    int routePos = std::distance(route.begin(), itr);
    auto stringPos = rng.randint(card);
    auto startIdx = routePos - stringPos;

    std::vector<int> indices;
    for (auto i = startIdx; i < startIdx + card; i++)
        indices.push_back(i % route.size());

    return indices;
}
Destroyed stringRemoval(Routes routes,
                        Client center,
                        Params const &params,
                        XorShift128 &rng)
{
    // Compute the maximum string cardinality to be removed
    size_t avgRouteSize = 0;
    for (auto &route : routes)
    {
        avgRouteSize += route.size();
    }
    avgRouteSize = avgRouteSize / routes.size();
    auto maxCard = std::min(params.config.maxStringCard, avgRouteSize);

    // Compute the number of strings to remove
    size_t nbStringRemovals
        = rng.randint(
              std::max(static_cast<size_t>(1),
                       (4 * params.config.avgDestruction) / (1 + maxCard) - 1))
          + 1;

    std::vector<Route> destroyedRoutes;  // TODO Use set instead?
    ClientSet removedClients;

    for (auto client : params.getNeighboursOf(center))
    {

        if (destroyedRoutes.size() >= nbStringRemovals)
            break;

        if (removedClients.contains(client))
            continue;

        for (auto &route : routes)
        {
            if (std::find(route.begin(), route.end(), client) != route.end())
            {
                if (std::find(
                        destroyedRoutes.begin(), destroyedRoutes.end(), route)
                    != destroyedRoutes.end())
                    continue;

                // Remove string from the route
                auto card = rng.randint(std::min(
                                static_cast<size_t>(route.size()), maxCard))
                            + 1;

                std::vector<int> removalIndices;
                if (rng.randint(100) <= params.config.splitRate)
                    removalIndices = selectString(route, client, card, rng);
                else
                {
                    int subSize = 1;
                    while (rng.randint(100) > params.config.splitDepth
                           and subSize < route.size() - card)
                    {
                        subSize++;
                    }

                    auto strIndices
                        = selectString(route, client, card + subSize, rng);
                    auto subPos = rng.randint(strIndices.size() - subSize);

                    removalIndices = strIndices;
                    for (auto i = 0; i < strIndices.size(); i++)
                    {
                        if (i < subPos or i >= subPos + card)
                            removalIndices.push_back(strIndices[i]);
                    }
                }

                // TODO Get rid of `removed`
                ClientSet removed;
                for (auto idx : removalIndices)
                    removed.insert(route[idx]);

                for (auto c : removed)
                {
                    std::vector<int>::iterator position
                        = std::find(route.begin(), route.end(), c);
                    route.erase(position);
                    removedClients.insert(c);
                }

                destroyedRoutes.push_back(route);
            }
        }

        return std::make_pair(routes, removedClients);
    }
}

int deltaCost(Client client, Client prev, Client next, Params const &params)
{
    int clientLate = params.clients[client].twLate;
    int distToInsert = params.dist(prev, client);
    int prevEarly = params.clients[prev].twEarly;

    if (prevEarly + distToInsert >= clientLate)
        return INT_MAX;

    int clientEarly = params.clients[client].twEarly;
    int distFromInsert = params.dist(client, next);
    int nextLate = params.clients[next].twLate;

    if (clientEarly + distFromInsert >= nextLate)
        return INT_MAX;

    return distToInsert + distFromInsert - params.dist(prev, next);
}

Individual greedyRepairWithBlinks(Routes &routes,
                                  ClientSet unplannedSet,
                                  Params const &params,
                                  XorShift128 &rng)

{
    // Sort clients
    std::vector unplanned(unplannedSet.begin(), unplannedSet.end());
    std::vector<int> indices(unplanned.size());
    std::iota(indices.begin(), indices.end(), 0);

    // TODO how to add more sorting options in a neat way?
    std::sort(indices.begin(), indices.end(), [&](int A, int B) -> bool {
        return params.clients[A].demand < params.clients[B].demand;
    });

    // NOTE Copied largely from SREX
    for (int idx : indices)
    {
        Client client = unplanned[idx];
        InsertPos best = {INT_MAX, &routes.front(), 0};

        for (auto &route : routes)
        {
            if (route.empty())
                break;

            for (size_t idx = 0; idx <= route.size(); ++idx)
            {
                if (rng.randint(100) > params.config.blinkRate)
                {
                    Client prev, next;

                    if (idx == 0)
                    {
                        prev = 0;
                        next = route[0];
                    }
                    else if (idx == route.size())
                    {
                        prev = route.back();
                        next = 0;
                    }
                    else
                    {
                        prev = route[idx - 1];
                        next = route[idx];
                    }

                    int const cost = deltaCost(client, prev, next, params);
                    if (cost < best.deltaCost)
                        best = {cost, &route, idx};
                }
            }
        }

        auto const [_, route, offset] = best;
        route->insert(route->begin() + static_cast<long>(offset), client);
    }
    return {&params, routes};
}
}  // namespace

Individual stringRemovalExchange(Parents const &parents,
                                 Params const &params,
                                 XorShift128 &rng)
{
    auto const &routes1 = parents.first->getRoutes();
    auto const &routes2 = parents.second->getRoutes();

    // Find a center node around which substrings will be removed
    Client center = rng.randint(params.nbClients) + 1;

    Destroyed destroyed1 = stringRemoval(routes1, center, params, rng);
    Destroyed destroyed2 = stringRemoval(routes2, center, params, rng);

    // Remove clients from other destroyed parent
    for (Client c : destroyed2.second)
    {
        for (auto &route : destroyed1.first)
        {
            if (std::find(route.begin(), route.end(), c) != route.end())
            {
                std::vector<int>::iterator position
                    = std::find(route.begin(), route.end(), c);
                route.erase(position);
                break;
            }
        }
    }

    for (Client c : destroyed1.second)
    {
        for (auto &route : destroyed2.first)
        {
            if (std::find(route.begin(), route.end(), c) != route.end())
            {
                std::vector<int>::iterator position
                    = std::find(route.begin(), route.end(), c);
                route.erase(position);
                break;
            }
        }
    }

    ClientSet removed;
    for (Client c : destroyed1.second)
        removed.insert(c);
    for (Client c : destroyed2.second)
        removed.insert(c);

    Individual indiv1
        = greedyRepairWithBlinks(destroyed1.first, removed, params, rng);
    Individual indiv2
        = greedyRepairWithBlinks(destroyed2.first, removed, params, rng);

    return std::min(indiv1, indiv2);
}
