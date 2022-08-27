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

    // Compute the broken pairs
    ClientSet brokenPairs;
    for (auto const [cust, succ] : succA)
        if (succ != succB[cust])
            brokenPairs.insert(cust);

    // TODO Do we only want to do 1 crossover?
    // * // Only repair remove/repair the worst individual
    // auto &worst = (parents.first > parents.second) ? routesA : routesB;

    // // Compute consecutive customers strings of broken pairs
    // std::shuffle(worst.begin(), worst.end(), rng);

    // ClientSet removed;
    // for (auto const &route : worst)
    // {
    //     for (auto const cust : route)
    //     {
    //         if (rng.randint(100) < 10 || removed.size() > maxNumRemovals)
    //             break;

    //         if (brokenPairs.contains(cust))
    //             removed.insert(cust);
    //     }
    // }

    // removeClients(worst, removed);
    // crossover::greedyRepair(worst, removed, params);
    // return {&params, worst};

    // Compute consecutive customers strings of broken pairs
    std::shuffle(routesA.begin(), routesA.end(), rng);

    ClientSet removedA;
    for (auto const &route : routesA)
    {
        for (auto const cust : route)
        {
            if (removedA.size() > maxNumRemovals)
                break;

            if (brokenPairs.contains(cust))
                removedA.insert(cust);
        }
    }

    removeClients(routesA, removedA);
    crossover::greedyRepair(routesA, removedA, params);
    Individual indivA{&params, routesA};

    // Do the same for routesB
    std::shuffle(routesB.begin(), routesB.end(), rng);

    ClientSet removedB;
    for (auto const &route : routesB)
    {
        for (auto const cust : route)
        {
            if (removedB.size() > maxNumRemovals)
                break;

            if (brokenPairs.contains(cust))
                removedB.insert(cust);
        }
    }

    removeClients(routesB, removedB);
    crossover::greedyRepair(routesB, removedB, params);
    Individual indivB{&params, routesB};

    return std::min(indivA, indivB);
}
