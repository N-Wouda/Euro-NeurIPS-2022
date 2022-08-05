#include "crossover.h"
#include "Params.h"

namespace
{
using Parents = std::pair<Individual const *, Individual const *>;
using Client = int;
using Clients = std::vector<Client>;
using ClientSet = std::unordered_set<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;

struct InsertPos  // best insert position, used to plan unplanned clients
{
    int deltaCost;
    std::vector<int> *route;
    size_t offset;
};

// Evaluates the cost change of inserting client between prev and next.
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
}  // namespace

void greedyRepairWithBlinks(Routes &routes,
                            Clients const unplanned,
                            size_t blinkRate,
                            Params const &params,
                            XorShift128 &rng)
{
    for (Client client : unplanned)
    {
        InsertPos best = {INT_MAX, &routes.front(), 0};

        for (auto &route : routes)
        {
            if (route.empty())
                continue;

            for (size_t idx = 0; idx <= route.size(); ++idx)
            {
                if (rng.randint(100) >= blinkRate)
                {
                    int prev, next;
                    if (idx
                        == 0)  // Currently depot -> [0]. Try depot -> client
                    {          // -> [0].
                        prev = 0;
                        next = route[0];
                    }
                    else if (idx
                             == route.size())  // Currently [-1] -> depot. Try
                    {                          // [-1] -> client -> depot.
                        prev = route.back();
                        next = 0;
                    }
                    else  // Currently [idx - 1] -> [idx]. Try [idx - 1] ->
                          // client
                    {     // -> [idx].
                        prev = route[idx - 1];
                        next = route[idx];
                    }
                    int const cost = deltaCost(client, prev, next, params);
                    if (cost < best.deltaCost)
                        best = {cost, &route, idx};
                }
            }
        }

        auto const [_, route, offset] = best;
        route->insert(route->begin() + static_cast<long>(offset), client);
    }
}
