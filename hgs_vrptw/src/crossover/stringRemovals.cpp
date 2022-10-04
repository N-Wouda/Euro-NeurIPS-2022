#include "crossover.h"

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <algorithm>
#include <functional>
#include <numeric>
#include <set>
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
    auto op = [&](size_t s, auto &r) { return s + r.size(); };
    size_t const avgRouteSize
        = std::accumulate(routes.begin(), routes.end(), 0, op) / routes.size();

    // Compute the maximum number of customers to remove
    size_t const nRemovals = params.config.destroyPct * params.nbClients / 100;

    std::set<Route> destroyedRoutes;
    ClientSet removedClients;

    auto neighbors = params.getNeighboursOf(center);
    std::shuffle(neighbors.begin(), neighbors.end(), rng);

    for (auto c : neighbors)
    {
        if (removedClients.size() >= nRemovals)
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
                = rng.randint(std::min(route.size(), avgRouteSize)) + 1;

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

}  // namespace

Individual stringRemovals(Individual &offspring,
                          Individual const &best,
                          Params const &params,
                          XorShift128 &rng)
{
    auto const routes = offspring.getRoutes();

    // Remove strings around a randomly picked center node
    Client const center = rng.randint(params.nbClients) + 1;
    auto [destroyed, removed] = stringRemoval(routes, center, params, rng);

    // TODO return removed as vector from stringRemoval
    auto removedVec = std::vector<Client>(removed.begin(), removed.end());

    crossover::greedyRepair(destroyed, removedVec, params);

    Individual indiv{&params, destroyed};
    return indiv;
}
