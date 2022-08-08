#include "operators.h"

#include "TimeWindowSegment.h"

bool moveTwoClients(Node *U, Node *V, Penalties const &penalties)
{
    using TWS = TimeWindowSegment;

    auto const &params = *U->params;

    if (U == n(V) || V == n(U) || n(U)->isDepot())
        return false;

    int const current
        = params.dist(p(U)->client, U->client, n(U)->client, nn(U)->client)
          + params.dist(V->client, n(V)->client);
    int const proposed
        = params.dist(V->client, U->client, n(U)->client, n(V)->client)
          + params.dist(p(U)->client, nn(U)->client);

    int deltaCost = proposed - current;

    if (U->route != V->route)
    {
        if (U->route->isFeasible() && deltaCost >= 0)
            return false;

        auto uTWS = TWS::merge(p(U)->twBefore, nn(U)->twAfter);

        deltaCost += penalties.timeWarp(uTWS);
        deltaCost -= penalties.timeWarp(U->route->tw);

        auto const uDemand = params.clients[U->client].demand;
        auto const xDemand = params.clients[n(U)->client].demand;

        deltaCost += penalties.load(U->route->load - uDemand - xDemand);
        deltaCost -= penalties.load(U->route->load);

        if (deltaCost >= 0)  // if delta cost of just U's route is not enough
            return false;    // even without V, the move will never be good

        deltaCost += penalties.load(V->route->load + uDemand + xDemand);
        deltaCost -= penalties.load(V->route->load);

        auto vTWS = TWS::merge(V->twBefore, U->tw, n(U)->tw, n(V)->twAfter);

        deltaCost += penalties.timeWarp(vTWS);
        deltaCost -= penalties.timeWarp(V->route->tw);
    }
    else  // within same route
    {
        if (!U->route->hasTimeWarp() && deltaCost >= 0)
            return false;

        if (U->position < V->position)
        {
            auto const uTWS = TWS::merge(p(U)->twBefore,
                                         Route::twBetween(nn(U), V),
                                         U->tw,
                                         n(U)->tw,
                                         n(V)->twAfter);

            deltaCost += penalties.timeWarp(uTWS);
        }
        else
        {
            auto const uTWS = TWS::merge(V->twBefore,
                                         U->tw,
                                         n(U)->tw,
                                         Route::twBetween(n(V), p(U)),
                                         nn(U)->twAfter);

            deltaCost += penalties.timeWarp(uTWS);
        }

        deltaCost -= penalties.timeWarp(U->route->tw);
    }

    if (deltaCost >= 0)
        return false;

    n(U)->insertAfter(V);
    U->insertAfter(V);

    return true;
}
