#include "operators.h"

#include "TimeWindowSegment.h"

int operators::singleMoveCost(Node *U, Node *V, Penalties const &penalties)
{
    using TWS = TimeWindowSegment;

    auto const &params = *U->params;

    if (U == n(V))  // move has no effect if U already comes directly after V
        return 0;

    int const current = params.dist(p(U)->client, U->client, n(U)->client)
                        + params.dist(V->client, n(V)->client);
    int const proposed = params.dist(V->client, U->client, n(V)->client)
                         + params.dist(p(U)->client, n(U)->client);

    int deltaCost = proposed - current;

    if (U->route != V->route)
    {
        if (U->route->isFeasible() && deltaCost >= 0)
            return deltaCost;

        auto const uTWS = TWS::merge(p(U)->twBefore, n(U)->twAfter);
        auto const vTWS = TWS::merge(V->twBefore, U->tw, n(V)->twAfter);
        auto const uDemand = params.clients[U->client].demand;

        deltaCost += penalties.load(U->route->load - uDemand)
                     + penalties.timeWarp(uTWS) - U->route->penalty
                     + penalties.load(V->route->load + uDemand)
                     + penalties.timeWarp(vTWS) - V->route->penalty;
    }
    else  // move within the same route
    {
        if (!U->route->hasTimeWarp() && deltaCost >= 0)
            return deltaCost;

        if (U->position < V->position)
        {
            auto const uTWS = TWS::merge(p(U)->twBefore,
                                         Route::twBetween(n(U), V),
                                         U->tw,
                                         n(V)->twAfter);

            deltaCost += penalties.timeWarp(uTWS);
        }
        else
        {
            auto const uTWS = TWS::merge(V->twBefore,
                                         U->tw,
                                         Route::twBetween(n(V), p(U)),
                                         n(U)->twAfter);

            deltaCost += penalties.timeWarp(uTWS);
        }

        deltaCost += penalties.load(U->route->load) - U->route->penalty;
    }

    return deltaCost;
}
