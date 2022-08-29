#include "crossover.h"
#include <algorithm>
#include <functional>
#include <unordered_set>

using Client = int;
using ClientSet = std::unordered_set<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;
using Successors = std::vector<Client>;

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
    auto &routesA = parents.first->getRoutes();
    auto &routesB = parents.second->getRoutes();

    // Find all successors and broken pairs
    // TODO check again based on what Niels said
    Successors succA;
    for (auto const &route : routesA)
        for (size_t idx = 0; idx < route.size(); idx++)
            succA[route[idx]] = route[idx + 1];

    Successors succB;
    for (auto const &route : routesB)
        for (size_t idx = 0; idx < route.size(); idx++)
            succB[route[idx]] = route[idx + 1];

    ClientSet brokenPairs;
    for (auto const client : succA)
        if (succA[client] != succB[client])
            brokenPairs.insert(client);

    size_t const maxNumRemovals
        = params.config.destroyPct * params.nbClients / 100;

    // Only consider the worst parent routes
    auto worst = parents.first > parents.second ? routesA : routesB;
    std::shuffle(worst.begin(), worst.end(), rng);

    ClientSet removed;
    for (auto const &route : worst)
        for (auto const client : route)
        {
            if (removed.size() >= maxNumRemovals)
                break;

            if (brokenPairs.contains(client))
                removed.insert(client);
        }

    removeClients(worst, removed);
    crossover::greedyRepair(worst, removed, params);
    return {&params, worst};
}
