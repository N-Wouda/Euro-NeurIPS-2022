#include "crossover.h"
#include "Params.h"

namespace
{
using Parents = std::pair<Individual const *, Individual const *>;
using Client = int;
using ClientSet = std::unordered_set<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;
}  // namespace

int deltaCost(Client client, Client prev, Client next, Params const &params)
{
    int clientLate = params.clients[client].twLate;
    int distToInsert = params.dist(prev, client);
    int prevEarly = params.clients[prev].twEarly;

    if (prevEarly + distToInsert >= clientLate)
        return INT_MAX;

    int clientEarly = params.clients[client].twEarly;
    int distFromInsert = params.dist(client, next);
    int nextLate = params.clients[next].twLate;

    if (clientEarly + distFromInsert >= nextLate)
        return INT_MAX;

    return distToInsert + distFromInsert - params.dist(prev, next);
}

std::pair<size_t, size_t> findPrevNext(Route const &route, size_t idx)
{
    int prev, next;
    if (idx == 0)  // Currently depot -> [0]. Try depot -> client
    {              // -> [0].
        prev = 0;
        next = route[0];
    }
    else if (idx == route.size())  // Currently [-1] -> depot. Try
    {                              // [-1] -> client -> depot.
        prev = route.back();
        next = 0;
    }
    else  // Currently [idx - 1] -> [idx]. Try [idx - 1] -> client
    {     // -> [idx].
        prev = route[idx - 1];
        next = route[idx];
    }
    return std::make_pair(prev, next);
}
