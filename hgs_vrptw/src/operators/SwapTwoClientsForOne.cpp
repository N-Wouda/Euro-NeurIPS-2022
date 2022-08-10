#include "SwapTwoClientsForOne.h"

#include "Route.h"
#include "TimeWindowSegment.h"

bool SwapTwoClientsForOne::test(Node *U, Node *V)
{
    using TWS = TimeWindowSegment;

    auto const &params = *U->params;

    if (U == p(V) || n(U) == p(V) || U == n(V) || n(U)->isDepot()
        || V->isDepot())
        return false;

    int const current = Route::distBetween(p(U), nn(U))
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

        deltaCost += d_penalties->timeWarp(uTWS);
        deltaCost -= d_penalties->timeWarp(U->route->tw);

        auto vTWS = TWS::merge(p(V)->twBefore, U->tw, n(U)->tw, n(V)->twAfter);

        deltaCost += d_penalties->timeWarp(vTWS);
        deltaCost -= d_penalties->timeWarp(V->route->tw);

        auto const vDemand = params.clients[V->client].demand;
        auto const loadDiff = Route::loadBetween(p(U), n(U)) - vDemand;

        deltaCost += d_penalties->load(U->route->load - loadDiff);
        deltaCost -= d_penalties->load(U->route->load);

        deltaCost += d_penalties->load(V->route->load + loadDiff);
        deltaCost -= d_penalties->load(V->route->load);
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

            deltaCost += d_penalties->timeWarp(uTWS);
        }
        else
        {
            auto const uTWS = TWS::merge(p(V)->twBefore,
                                         U->tw,
                                         n(U)->tw,
                                         Route::twBetween(n(V), p(U)),
                                         V->tw,
                                         nn(U)->twAfter);

            deltaCost += d_penalties->timeWarp(uTWS);
        }

        deltaCost -= d_penalties->timeWarp(U->route->tw);
    }

    return deltaCost < 0;
}
