#include "crossover.h"
#include <algorithm>
#include <functional>
#include <unordered_set>

using Client = int;
using Clients = std::vector<Client>;
using ClientSet = std::unordered_set<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;

namespace
{
void removeClients(Routes &routes, ClientSet const &clients)
{
    std::vector<Client> orderedClients;
    std::vector<std::reference_wrapper<Route>> clientRoutes;

    for (auto &route : routes)
        for (Client client : route)
            if (clients.contains(client))
            {
                orderedClients.push_back(client);
                clientRoutes.push_back(std::ref(route));
            }

    for (size_t idx = 0; idx < orderedClients.size(); idx++)
    {
        auto client = orderedClients[idx];
        auto &route = clientRoutes[idx].get();
        auto const position = std::find(route.begin(), route.end(), client);
        route.erase(position);
    }
}
}  // namespace

Individual brokenPairsExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng)
{
    auto const &routesA = parents.first->getRoutes();
    auto const &routesB = parents.second->getRoutes();

    // Find all broken pairs between the two parents
    Clients succA(params.nbClients + 1, 0);
    for (auto const &route : routesA)
        for (size_t idx = 1; idx < route.size(); idx++)
            succA[route[idx - 1]] = route[idx];

    Clients succB(params.nbClients + 1, 0);
    for (auto const &route : routesB)
        for (size_t idx = 1; idx < route.size(); idx++)
            succB[route[idx - 1]] = route[idx];

    ClientSet brokenPairs;
    for (Client client = 1; client <= params.nbClients; client++)
        if (succA[client] != succB[client])
            brokenPairs.insert(client);

    size_t const maxNumRemovals
        = params.config.destroyPct * params.nbClients / 100;

    // Only destroy-and-repair the worst parent routes
    auto worst = parents.first > parents.second ? routesA : routesB;
    auto const numRoutes = parents.first > parents.second
                               ? parents.first->numRoutes()
                               : parents.second->numRoutes();

    // Go through the non-empty routes in random order
    std::shuffle(worst.begin(), worst.begin() + numRoutes, rng);

    // Remove consecutive broken pairs
    ClientSet removed;
    removed.reserve(maxNumRemovals);

    for (auto const &route : worst)
    {
        if (route.empty())
            break;

        for (auto const client : route)
        {
            if (removed.size() >= maxNumRemovals)
                break;

            if (brokenPairs.contains(client))
                removed.insert(client);
        }
    }

    removeClients(worst, removed);
    crossover::greedyRepair(worst, removed, params);
    return {&params, worst};
}
