#include "SwapTwoClientPairs.h"

#include "Route.h"
#include "TimeWindowSegment.h"

bool SwapTwoClientPairs::test(Node *U, Node *V)
{
    using TWS = TimeWindowSegment;

    auto const &params = *U->params;

    if (U->client >= V->client)
        return false;

    if (n(U)->isDepot() || n(V)->isDepot() || n(V) == p(U) || U == n(V)
        || n(U) == V || V == nn(U) || V->isDepot())
        return false;

    int const current
        = Route::distBetween(p(U), nn(U)) + Route::distBetween(p(V), nn(V));

    int const proposed
        = params.dist(p(U)->client, V->client, n(V)->client, nn(U)->client)
          + params.dist(p(V)->client, U->client, n(U)->client, nn(V)->client);

    int deltaCost = proposed - current;

    if (U->route != V->route)
    {
        if (U->route->isFeasible() && V->route->isFeasible() && deltaCost >= 0)
            return false;

        auto uTWS = TWS::merge(p(U)->twBefore, V->tw, n(V)->tw, nn(U)->twAfter);

        deltaCost += d_penalties->timeWarp(uTWS);
        deltaCost -= d_penalties->timeWarp(U->route->tw);

        auto vTWS = TWS::merge(p(V)->twBefore, U->tw, n(U)->tw, nn(V)->twAfter);

        deltaCost += d_penalties->timeWarp(vTWS);
        deltaCost -= d_penalties->timeWarp(V->route->tw);

        auto const uDemand = params.clients[U->client].demand;
        auto const xDemand = params.clients[n(U)->client].demand;
        auto const vDemand = params.clients[V->client].demand;
        auto const yDemand = params.clients[n(V)->client].demand;
        auto const loadDiff = uDemand + xDemand - vDemand - yDemand;

        deltaCost += d_penalties->load(U->route->load - loadDiff);
        deltaCost -= d_penalties->load(U->route->load);

        deltaCost += d_penalties->load(V->route->load + loadDiff);
        deltaCost -= d_penalties->load(V->route->load);
    }
    else  // swap within the same route
    {
        if (!U->route->hasTimeWarp() && deltaCost >= 0)
            return false;

        if (U->position < V->position)
        {
            auto const uTWS = TWS::merge(p(U)->twBefore,
                                         V->tw,
                                         n(V)->tw,
                                         Route::twBetween(nn(U), p(V)),
                                         U->tw,
                                         n(U)->tw,
                                         nn(V)->twAfter);

            deltaCost += d_penalties->timeWarp(uTWS);
        }
        else
        {
            auto const uTWS = TWS::merge(p(V)->twBefore,
                                         U->tw,
                                         n(U)->tw,
                                         Route::twBetween(nn(V), p(U)),
                                         V->tw,
                                         n(V)->tw,
                                         nn(U)->twAfter);

            deltaCost += d_penalties->timeWarp(uTWS);
        }

        deltaCost -= d_penalties->timeWarp(U->route->tw);
    }

    return deltaCost < 0;
}
