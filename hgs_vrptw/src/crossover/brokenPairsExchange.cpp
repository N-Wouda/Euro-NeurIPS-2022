#include "crossover.h"

using Client = int;
using Clients = std::vector<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;

Individual brokenPairsExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng)
{
    auto const &neighboursA = parents.first->getNeighbours();
    auto const &neighboursB = parents.second->getNeighbours();

    Clients brokenPairs;
    for (Client client = 1; client <= params.nbClients; client++)
        // Test if client has a different successor in each individual
        if (neighboursA[client].second != neighboursB[client].second)
            brokenPairs.push_back(client);

    // Only destroy-and-repair the parent's routes whose cost is greatest
    // TODO why worst? Why not best? Why not both?
    Routes worst = std::max(parents.first, parents.second)->getRoutes();
    std::vector<Route *> client2route(params.nbClients + 1, nullptr);

    for (auto &route : worst)
        for (Client client : route)
            client2route[client] = &route;

    // Shuffle the broken pairs, and then truncate to limit destruction
    std::shuffle(brokenPairs.begin(), brokenPairs.end(), rng);
    size_t const nRemovals = params.config.destroyPct * params.nbClients / 100;
    brokenPairs.resize(std::min(nRemovals, brokenPairs.size()));

    for (Client client : brokenPairs)  // remove
    {
        auto *route = client2route[client];
        auto const position = std::find(route->begin(), route->end(), client);
        route->erase(position);
    }

    crossover::greedyRepair(worst, brokenPairs, params);  // repair

    return {&params, worst};
}
