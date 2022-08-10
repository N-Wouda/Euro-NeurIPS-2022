#include "operators.h"

#include "TimeWindowSegment.h"

bool swapTwoClientsForOne(Node *U, Node *V, Penalties const &penalties)
{
    using TWS = TimeWindowSegment;

    auto const &params = *U->params;

    if (U == p(V) || n(U) == p(V) || U == n(V) || n(U)->isDepot()
        || V->isDepot())
        return false;

    int const current
        = params.dist(p(U)->client, U->client, n(U)->client, nn(U)->client)
          + params.dist(p(V)->client, V->client, n(V)->client);
    int const proposed
        = params.dist(p(U)->client, V->client, nn(U)->client)
          + params.dist(p(V)->client, U->client, n(U)->client, n(V)->client);

    int deltaCost = proposed - current;

    if (U->route != V->route)
    {
        if (U->route->isFeasible() && V->route->isFeasible() && deltaCost >= 0)
            return false;

        auto uTWS = TWS::merge(p(U)->twBefore, V->tw, nn(U)->twAfter);

        deltaCost += penalties.timeWarp(uTWS);
        deltaCost -= penalties.timeWarp(U->route->tw);

        auto vTWS = TWS::merge(p(V)->twBefore, U->tw, n(U)->tw, n(V)->twAfter);

        deltaCost += penalties.timeWarp(vTWS);
        deltaCost -= penalties.timeWarp(V->route->tw);

        auto const uDemand = params.clients[U->client].demand;
        auto const xDemand = params.clients[n(U)->client].demand;
        auto const vDemand = params.clients[V->client].demand;
        auto const loadDiff = uDemand + xDemand - vDemand;

        deltaCost += penalties.load(U->route->load - loadDiff);
        deltaCost -= penalties.load(U->route->load);

        deltaCost += penalties.load(V->route->load + loadDiff);
        deltaCost -= penalties.load(V->route->load);
    }
    else  // within same route
    {
        if (!U->route->hasTimeWarp() && deltaCost >= 0)
            return false;

        if (U->position < V->position)
        {
            auto const uTWS = TWS::merge(p(U)->twBefore,
                                         V->tw,
                                         Route::twBetween(nn(U), p(V)),
                                         U->tw,
                                         n(U)->tw,
                                         n(V)->twAfter);

            deltaCost += penalties.timeWarp(uTWS);
        }
        else
        {
            auto const uTWS = TWS::merge(p(V)->twBefore,
                                         U->tw,
                                         n(U)->tw,
                                         Route::twBetween(n(V), p(U)),
                                         V->tw,
                                         nn(U)->twAfter);

            deltaCost += penalties.timeWarp(uTWS);
        }

        deltaCost -= penalties.timeWarp(U->route->tw);
    }

    if (deltaCost >= 0)
        return false;

    n(U)->insertAfter(V);
    U->swapWith(V);

    return true;
}
