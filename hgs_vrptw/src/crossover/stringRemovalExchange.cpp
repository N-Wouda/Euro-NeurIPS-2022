#include "crossover.h"

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"
#include <unordered_set>

namespace
{
using Parents = std::pair<Individual const *, Individual const *>;
using Client = int;
using ClientSet = std::unordered_set<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;
using Destroyed = std::pair<Routes, ClientSet>;

Destroyed stringRemoval(Routes routes,
                        Client center,
                        Params const &params,
                        XorShift128 &rng)
{
    ClientSet removed;
    return std::make_pair(routes, removed);
};

Individual greedyRepairWithBlinks(Destroyed destroyed,
                                  Params const &params,
                                  XorShift128 &rng)
{
    return {&params, destroyed.first};
}
}  // namespace

Individual stringRemovalExchange(Parents const &parents,
                                 Params const &params,
                                 XorShift128 &rng)
{
    auto const &routes1 = parents.first->getRoutes();
    auto const &routes2 = parents.second->getRoutes();

    // Find a center node around which substrings will be removed
    Client center = rng.randint(params.nbClients) + 1;

    Destroyed destroyed1 = stringRemoval(routes1, center, params, rng);

    Individual indiv1 = greedyRepairWithBlinks(destroyed1, params, rng);

    return indiv1;
}
