#include "crossover.h"

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <algorithm>
#include <functional>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace
{
using Parents = std::pair<Individual const *, Individual const *>;
using Client = int;
using ClientSet = std::unordered_set<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;

// Returns the indices of a random (sub)string that contains the client
std::vector<size_t> selectString(Route const &route,
                                 Client const client,
                                 size_t const stringSize,
                                 XorShift128 &rng)
{
    auto itr = std::find(route.begin(), route.end(), client);
    auto routePos = std::distance(route.begin(), itr);
    auto stringPos = rng.randint(stringSize);
    auto startIdx = routePos - stringPos;

    std::vector<size_t> indices;
    for (size_t i = startIdx; i != startIdx + stringSize; i++)
        indices.push_back(i % route.size());

    return indices;
}

// Removes a number of strings around the center client
std::pair<Routes, ClientSet> stringRemoval(Routes routes,
                                           Client const center,
                                           Params const &params,
                                           XorShift128 &rng)
{
    auto op = [&](size_t s, auto &r) { return s + r.size(); };
    size_t avgRouteSize
        = std::accumulate(routes.begin(), routes.end(), 0, op) / routes.size();
    auto const maxSize = std::min(params.config.maxStringSize, avgRouteSize);

    // Compute the number of strings to remove
    auto const nbStringRemovals
        = rng.randint(params.config.maxStringRemovals) + 1;

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
            if (std::find(route.begin(), route.end(), client) == route.end())
                continue;

            if (destroyedRoutes.contains(route))
                continue;

            // Remove string from the route
            auto const stringSize
                = rng.randint(std::min(route.size(), maxSize)) + 1;

            std::vector<size_t> removalIndices;

            if (rng.randint(100) >= params.config.splitRate)
                removalIndices = selectString(route, client, stringSize, rng);
            else
            {
                size_t subSize = 1;
                while (rng.randint(100) >= params.config.splitDepth
                       and subSize < route.size() - stringSize)
                    subSize++;

                auto const strIndices
                    = selectString(route, client, stringSize + subSize, rng);
                auto const subPos
                    = rng.randint(strIndices.size() - subSize + 1);

                for (size_t i = 0; i < subPos; i++)
                    removalIndices.push_back(strIndices[i]);

                for (auto i = subPos + stringSize; i < strIndices.size(); i++)
                    removalIndices.push_back(strIndices[i]);
            }

            std::sort(
                removalIndices.begin(), removalIndices.end(), std::greater<>());

            for (auto idx : removalIndices)
            {
                removedClients.insert(route[idx]);
                route.erase(route.begin() + idx);
            }

            destroyedRoutes.insert(route);
            break;
        }
    }

    return std::make_pair(routes, removedClients);
}

void removeClients(Routes &routes, ClientSet const &clients)
{
    std::vector<Client> orderedClients;
    std::vector<std::reference_wrapper<Route>> clientRoutes;

    for (auto &route : routes)
        for (Client c : route)
            if (clients.contains(c))
            {
                orderedClients.push_back(c);
                clientRoutes.push_back(std::ref(route));
            }

    for (size_t i = 0; i < orderedClients.size(); i++)
    {
        auto client = orderedClients[i];
        auto &route = clientRoutes[i].get();
        auto const position = std::find(route.begin(), route.end(), client);
        route.erase(position);
    }
}

std::vector<int>
sortClients(ClientSet const &clientSet, Params const &params, XorShift128 &rng)
{
    std::vector clients(clientSet.begin(), clientSet.end());
    std::vector<int> indices(clients.size());
    std::iota(indices.begin(), indices.end(), 0);

    auto const highestDemand = [&](int A, int B) {
        return params.clients[A].demand > params.clients[B].demand;
    };

    auto const furtherToDepot
        = [&](int A, int B) { return params.dist(0, A) > params.dist(0, B); };

    auto const closestToDepot
        = [&](int A, int B) { return params.dist(0, A) < params.dist(0, B); };

    auto const largestTw = [&](int A, int B) {
        return params.clients[A].twLate - params.clients[A].twEarly
               > params.clients[B].twLate - params.clients[B].twEarly;
    };

    auto const smallestTwEarly = [&](int A, int B) {
        return params.clients[A].twEarly < params.clients[B].twEarly;
    };

    auto const smallestTwLate = [&](int A, int B) {
        return params.clients[A].twLate < params.clients[B].twEarly;
    };

    // TODO How to make this non-uniform?
    auto const draw = rng.randint(7);
    if (draw == 1)
        std::shuffle(indices.begin(), indices.end(), rng);
    else if (draw == 2)
        std::sort(indices.begin(), indices.end(), highestDemand);
    else if (draw == 3)
        std::sort(indices.begin(), indices.end(), furtherToDepot);
    else if (draw == 4)
        std::sort(indices.begin(), indices.end(), closestToDepot);
    else if (draw == 5)
        std::sort(indices.begin(), indices.end(), largestTw);
    else if (draw == 6)
        std::sort(indices.begin(), indices.end(), smallestTwEarly);
    else if (draw == 7)
        std::sort(indices.begin(), indices.end(), smallestTwLate);

    std::vector<int> sortedClients;
    for (auto idx : indices)
        sortedClients.push_back(clients[idx]);

    return sortedClients;
}
}  // namespace

Individual stringRemovalExchange(Parents const &parents,
                                 Params const &params,
                                 XorShift128 &rng)
{
    auto const &routes1 = parents.first->getRoutes();
    auto const &routes2 = parents.second->getRoutes();

    // Find a center node around which strings will be removed
    Client const center = rng.randint(params.nbClients) + 1;

    auto [destroyed1, removed1] = stringRemoval(routes1, center, params, rng);
    auto [destroyed2, removed2] = stringRemoval(routes2, center, params, rng);

    // Remove clients from other destroyed parent
    removeClients(destroyed1, removed2);
    removeClients(destroyed2, removed1);

    auto removedSet = removed1;
    removedSet.insert(removed2.begin(), removed2.end());
    auto const removed = sortClients(removedSet, params, rng);

    crossover::greedyRepair(destroyed1, removed, params);
    crossover::greedyRepair(destroyed2, removed, params);

    Individual indiv1{&params, destroyed1};
    Individual indiv2{&params, destroyed2};

    return std::min(indiv1, indiv2);
}
