#include "crossover.h"
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

    // Find client successors in both parents
    Successors succA;
    for (auto const &route : routesA)
        for (int idx = 0; idx <= static_cast<int>(route.size()) - 1; idx++)
            succA[route[idx]] = route[idx + 1];

    Successors succB;
    for (auto const &route : routesB)
        for (int idx = 0; idx <= static_cast<int>(route.size()) - 1; idx++)
            succB[route[idx]] = route[idx + 1];

    // Compute number of broken pairs
    size_t nBrokenPairs = 0;
    for (int idx = 0; idx != params.nbClients; idx++)
        nBrokenPairs += succA[idx] == succB[idx] ? 1 : 0;

    // Compute the maximum number of client removals
    size_t const maxNumRemovals
        = params.config.destructionRate * params.nbClients;

    // Store the clients to remove
    ClientSet removeA;
    for (auto const [cust, succ] : succA)
    {
        if (removeA.size() > maxNumRemovals)
            break;

        if (succ != succB[cust])
            removeA.insert(cust);
    }

    ClientSet removeB;
    for (auto const [cust, succ] : succB)
    {
        if (removeB.size() > maxNumRemovals)
            break;

        if (succ != succA[cust])
            removeB.insert(cust);
    }

    // Remove customers from both parents
    removeClients(routesA, removeA);
    removeClients(routesB, removeB);

    // Repair both parents
    crossover::greedyRepair(routesA, removeA, params);
    crossover::greedyRepair(routesB, removeB, params);

    Individual indivA{&params, routesA};
    Individual indivB{&params, routesB};

    return std::min(indivA, indivB);
}
