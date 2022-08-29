#include "crossover.h"
#include <algorithm>
#include <functional>
#include <unordered_map>
#include <unordered_set>

using Client = int;
using ClientSet = std::unordered_set<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;
using Successors = std::unordered_map<int, int>;

namespace
{
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

Individual brokenPairsExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng)
{
    auto routesA = parents.first->getRoutes();
    auto routesB = parents.second->getRoutes();

    // Find all successors and broken pairs
    Successors succA;
    for (auto const &route : routesA)
        for (int idx = 0; idx <= static_cast<int>(route.size()) - 1; idx++)
            succA[route[idx]] = route[idx + 1];

    Successors succB;
    for (auto const &route : routesB)
        for (int idx = 0; idx <= static_cast<int>(route.size()) - 1; idx++)
            succB[route[idx]] = route[idx + 1];

    ClientSet brokenPairs;
    for (auto const [cust, succ] : succA)
        if (succ != succB[cust])
            brokenPairs.insert(cust);

    size_t const maxNumRemovals
        = params.config.destructionRate * params.nbClients;

    // Only consider the worst parent routes
    auto &worst = (parents.first > parents.second) ? routesA : routesB;
    std::shuffle(worst.begin(), worst.end(), rng);

    ClientSet removed;
    for (auto const &route : worst)
    {
        for (auto const cust : route)
        {
            if (removed.size() >= maxNumRemovals)
                break;

            if (brokenPairs.contains(cust))
                removed.insert(cust);
        }
    }

    removeClients(worst, removed);
    crossover::greedyRepair(worst, removed, params);
    return {&params, worst};
}
