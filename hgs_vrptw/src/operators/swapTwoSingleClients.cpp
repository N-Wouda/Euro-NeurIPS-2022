#include "operators.h"

bool swapTwoSingleClients(Node *U, Node *V, Penalties const &penalties)
{
    using TWS = TimeWindowSegment;

    auto const &params = *U->params;

    if (U->client >= V->client || U == p(V) || U == n(V))
        return false;

    int const current = params.dist(p(U)->client, U->client, n(U)->client)
                        + params.dist(p(V)->client, V->client, n(V)->client);
    int const proposed = params.dist(p(U)->client, V->client, n(U)->client)
                         + params.dist(p(V)->client, U->client, n(V)->client);

    int deltaCost = proposed - current;

    if (U->route != V->route)
    {
        if (U->route->isFeasible() && V->route->isFeasible() && deltaCost >= 0)
            return false;

        auto uTWS = TWS::merge(p(U)->twBefore, V->tw, n(U)->twAfter);
        auto vTWS = TWS::merge(p(V)->twBefore, U->tw, n(V)->twAfter);

        auto const uDemand = params.clients[U->client].demand;
        auto const vDemand = params.clients[V->client].demand;

        deltaCost += penalties.load(U->route->load + vDemand - uDemand)
                     + penalties.timeWarp(uTWS) - U->route->penalty
                     + penalties.load(V->route->load + uDemand - vDemand)
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
                                         Route::twBetween(n(U), p(V)),
                                         U->tw,
                                         n(V)->twAfter);

            deltaCost += penalties.timeWarp(uTWS);
        }
        else
        {
            auto const uTWS = TWS::merge(p(V)->twBefore,
                                         U->tw,
                                         Route::twBetween(n(V), p(U)),
                                         V->tw,
                                         n(U)->twAfter);

            deltaCost += penalties.timeWarp(uTWS);
        }

        deltaCost += penalties.load(U->route->load) - U->route->penalty;
    }

    if (deltaCost >= 0)
        return false;

    U->swapWith(V);

    return true;
}
