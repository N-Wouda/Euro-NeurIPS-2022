#include "crossover.h"

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <algorithm>
#include <functional>
#include <numeric>
#include <unordered_set>

namespace
{
using Parents = std::pair<Individual const *, Individual const *>;
using Client = int;
using ClientSet = std::unordered_set<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;

size_t mod(int x, int N) { return (x % N + N) % N; }

// Returns the indices of a string that contains the client
std::vector<size_t> selectString(Route const &route,
                                 Client const client,
                                 size_t const stringSize,
                                 XorShift128 &rng)
{
    auto itr = std::find(route.begin(), route.end(), client);
    auto routePos = std::distance(route.begin(), itr);
    auto stringPos = rng.randint(stringSize);
    auto startIdx = routePos - stringPos;  // can become negative

    std::vector<size_t> indices;
    for (int i = startIdx; i != startIdx + static_cast<int>(stringSize); i++)
        indices.push_back(mod(i, route.size()));

    return indices;
}

// Removes a number of strings around the center client
std::pair<Routes, ClientSet> stringRemoval(Routes routes,
                                           Client const center,
                                           Params const &params,
                                           XorShift128 &rng)
{
    // Normalize maxStringSize for the current route characteristics
    auto op = [&](size_t s, auto &r) { return s + r.size(); };
    size_t avgRouteSize
        = std::accumulate(routes.begin(), routes.end(), 0, op) / routes.size();
    auto const maxStringSize
        = std::min(params.config.maxStringSize, avgRouteSize);

    // Compute the number of strings to remove
    auto const nbStringRemovals
        = rng.randint(params.config.maxStringRemovals) + 1;

    std::set<Route> destroyedRoutes;
    ClientSet removedClients;

    auto neighbors = params.getNeighboursOf(center);
    std::shuffle(neighbors.begin(), neighbors.end(), rng);

    for (auto c : neighbors)
    {
        if (destroyedRoutes.size() >= nbStringRemovals)
            break;

        if (removedClients.contains(c))
            continue;

        for (auto &route : routes)
        {
            if (std::find(route.begin(), route.end(), c) == route.end())
                continue;

            if (destroyedRoutes.contains(route))
                continue;

            auto const stringSize
                = rng.randint(std::min(route.size(), maxStringSize)) + 1;

            // Find the route indices of the string to be removed
            auto indices = selectString(route, c, stringSize, rng);

            for (auto idx : indices)
                removedClients.insert(route[idx]);

            std::sort(indices.begin(), indices.end(), std::greater<>());

            for (auto idx : indices)
                route.erase(route.begin() + idx);

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

    auto removed = std::vector<Client>(removed1.begin(), removed2.end());
    for (Client c : removed2)
        if (!removed1.contains(c))
            removed.push_back(c);

    crossover::greedyRepair(destroyed1, removed, params, rng);
    crossover::greedyRepair(destroyed2, removed, params, rng);

    Individual indiv1{&params, destroyed1};
    Individual indiv2{&params, destroyed2};

    return std::min(indiv1, indiv2);
}
