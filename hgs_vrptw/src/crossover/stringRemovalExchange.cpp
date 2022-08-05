#include "crossover.h"

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <numeric>
#include <string>
#include <unordered_set>

namespace
{
using Parents = std::pair<Individual const *, Individual const *>;
using Client = int;
using Clients = std::vector<Client>;
using ClientSet = std::unordered_set<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;

;

// Returns the indices of a random (sub)string with length card that
// contains the client
std::vector<size_t>
selectString(Route const &route, Client client, size_t card, XorShift128 &rng)
{
    auto itr = std::find(route.begin(), route.end(), client);
    auto routePos = std::distance(route.begin(), itr);
    auto stringPos = rng.randint(card);
    auto startIdx = routePos - stringPos;

    std::vector<size_t> indices;
    for (auto i = startIdx; i != startIdx + card; i++)
        indices.push_back(i % route.size());

    return indices;
}

std::pair<Routes, ClientSet> stringRemoval(Routes routes,
                                           Client center,
                                           Params const &params,
                                           XorShift128 &rng)
{
    // Compute the maximum string cardinality to be removed
    size_t avgRouteSize = 0;
    for (auto &route : routes)
        avgRouteSize += route.size();
    avgRouteSize = avgRouteSize / routes.size();
    auto maxCard = std::min(params.config.maxStringCard, avgRouteSize);

    // Compute the number of strings to remove
    // NOTE Deviates from original because we use discrete unif distribution
    auto maxStringRemovals = (4 * params.config.avgDestruction) / (1 + maxCard);
    auto nbStringRemovals = rng.randint(maxStringRemovals) + 1;

    std::set<Route> destroyedRoutes;
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
                if (destroyedRoutes.contains(route))
                    continue;

                // Remove string from the route
                auto card = rng.randint(std::min(
                                static_cast<size_t>(route.size()), maxCard))
                            + 1;

                std::vector<size_t> removalIndices;
                if (rng.randint(100) <= params.config.splitRate)
                    removalIndices = selectString(route, client, card, rng);
                else
                {
                    auto subSize = 1;
                    while (rng.randint(100) > params.config.splitDepth
                           and subSize < route.size() - card)
                        subSize++;

                    auto strIndices
                        = selectString(route, client, card + subSize, rng);
                    auto subPos = rng.randint(strIndices.size() - subSize + 1);

                    for (auto i = 0; i <= subPos; i++)
                        removalIndices.push_back(strIndices[i]);
                    for (auto i = subPos + card; i <= strIndices.size(); i++)
                        removalIndices.push_back(strIndices[i]);

                    removalIndices = selectString(route, client, card, rng);
                }

                std::sort(removalIndices.begin(),
                          removalIndices.end(),
                          std::greater<>());

                for (auto idx : removalIndices)
                {
                    removedClients.insert(route[idx]);
                    route.erase(route.begin() + idx);
                }

                destroyedRoutes.insert(route);
            }
        }

        return std::make_pair(routes, removedClients);
    }
}

void removeClients(Routes &routes, ClientSet const &clients)
{
    for (Client c : clients)
    {
        for (auto &route : routes)
        {
            // TODO Make this more efficient
            if (std::find(route.begin(), route.end(), c) != route.end())
            {
                auto position = std::find(route.begin(), route.end(), c);
                route.erase(position);
                break;
            }
        }
    }
}
Clients
sortClients(ClientSet const clientSet, Params const &params, XorShift128 &rng)
{
    std::vector clients(clientSet.begin(), clientSet.end());
    std::vector<int> indices(clients.size());
    std::iota(indices.begin(), indices.end(), 0);

    // TODO how to add more sorting options in a neat way?
    std::sort(indices.begin(),
              indices.end(),
              [&](int A, int B)
              { return params.clients[A].demand < params.clients[B].demand; });

    std::vector<int> sortedClients;
    for (auto idx : indices)
        sortedClients.push_back(clients[idx]);

    return sortedClients;
}

Individual greedyRepairWithBlinks(Routes &routes,
                                  ClientSet const unplannedSet,
                                  Params const &params,
                                  XorShift128 &rng)

{
    auto unplanned = sortClients(unplannedSet, params, rng);

    for (Client client : unplanned)
    {
        InsertPos best = {INT_MAX, &routes.front(), 0};

        for (auto &route : routes)
        {
            // NOTE Perhaps we should consider empty routes
            if (route.empty())
                continue;

            for (size_t idx = 0; idx <= route.size(); ++idx)
            {
                if (rng.randint(100) >= params.config.blinkRate)
                {
                    auto const [prev, next] = findPrevNext(route, idx);

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
    Client const center = rng.randint(params.nbClients) + 1;

    auto [destroyed1, removed1] = stringRemoval(routes1, center, params, rng);
    auto [destroyed2, removed2] = stringRemoval(routes2, center, params, rng);

    // Remove clients from other destroyed parent
    removeClients(destroyed1, removed2);
    removeClients(destroyed2, removed1);

    auto removed = removed1;
    removed.insert(removed2.begin(), removed2.end());

    Individual indiv1
        = greedyRepairWithBlinks(destroyed1, removed, params, rng);
    Individual indiv2
        = greedyRepairWithBlinks(destroyed2, removed, params, rng);

    return std::min(indiv1, indiv2);
}
