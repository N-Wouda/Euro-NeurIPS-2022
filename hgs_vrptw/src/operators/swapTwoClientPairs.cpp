#include "operators.h"

#include "TimeWindowSegment.h"

bool swapTwoClientPairs(Node *U, Node *V, Penalties const &penalties)
{
    using TWS = TimeWindowSegment;

    auto const &params = *U->params;

    if (U->client >= V->client)
        return false;

    if (n(U)->isDepot || n(V)->isDepot || n(V) == p(U) || U == n(V) || n(U) == V
        || V == nn(U))
        return false;

    int const current
        = params.dist(p(U)->client, U->client, n(U)->client, nn(U)->client)
          + params.dist(p(V)->client, V->client, n(V)->client, nn(V)->client);

    int const proposed
        = params.dist(p(U)->client, V->client, n(V)->client, nn(U)->client)
          + params.dist(p(V)->client, U->client, n(U)->client, nn(V)->client);

    int deltaCost = proposed - current;

    if (U->route != V->route)
    {
        if (U->route->isFeasible() && V->route->isFeasible() && deltaCost >= 0)
            return false;

        auto uTWS = TWS::merge(p(U)->twBefore, V->tw, n(V)->tw, nn(U)->twAfter);
        auto vTWS = TWS::merge(p(V)->twBefore, U->tw, n(U)->tw, nn(V)->twAfter);

        auto const uDemand = params.clients[U->client].demand;
        auto const xDemand = params.clients[n(U)->client].demand;
        auto const vDemand = params.clients[V->client].demand;
        auto const yDemand = params.clients[n(V)->client].demand;
        auto const loadDiff = uDemand + xDemand - vDemand - yDemand;

        deltaCost += penalties.load(U->route->load - loadDiff)
                     + penalties.timeWarp(uTWS) - U->route->penalty
                     + penalties.load(V->route->load + loadDiff)
                     + penalties.timeWarp(vTWS) - V->route->penalty;
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

            deltaCost += penalties.timeWarp(uTWS);
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

            deltaCost += penalties.timeWarp(uTWS);
        }

        deltaCost += penalties.load(U->route->load) - U->route->penalty;
    }

    if (deltaCost >= 0)
        return false;

    n(U)->swapWith(n(V));
    U->swapWith(V);

    return true;
}
