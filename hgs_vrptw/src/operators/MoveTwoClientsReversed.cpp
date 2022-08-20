#include "MoveTwoClientsReversed.h"

#include "Route.h"
#include "TimeWindowSegment.h"

int MoveTwoClientsReversed::evaluate(Node *U, Node *V)
{
    using TWS = TimeWindowSegment;

    if (U == n(V) || n(U) == V || n(U)->isDepot())
        return 0;

    int const current = Route::distBetween(p(U), nn(U))
                        + d_params.dist(V->client, n(V)->client);
    int const proposed
        = d_params.dist(p(U)->client, nn(U)->client)
          + d_params.dist(V->client, n(U)->client, U->client, n(V)->client);

    int deltaCost = proposed - current;

    if (U->route != V->route)
    {
        if (U->route->isFeasible() && deltaCost >= 0)
            return deltaCost;

        auto uTWS = TWS::merge(p(U)->twBefore, nn(U)->twAfter);

        deltaCost += d_penalties->timeWarp(uTWS);
        deltaCost -= d_penalties->timeWarp(U->route->tw);

        auto const loadDiff = Route::loadBetween(U, n(U));

        deltaCost += d_penalties->load(U->route->load() - loadDiff);
        deltaCost -= d_penalties->load(U->route->load());

        if (deltaCost >= 0)    // if delta cost of just U's route is not enough
            return deltaCost;  // even without V, the move will never be good

        deltaCost += d_penalties->load(V->route->load() + loadDiff);
        deltaCost -= d_penalties->load(V->route->load());

        auto vTWS = TWS::merge(V->twBefore, n(U)->tw, U->tw, n(V)->twAfter);

        deltaCost += d_penalties->timeWarp(vTWS);
        deltaCost -= d_penalties->timeWarp(V->route->tw);
    }
    else  // within same route
    {
        if (!U->route->hasTimeWarp() && deltaCost >= 0)
            return deltaCost;

        if (U->position < V->position)
        {
            auto const uTWS = TWS::merge(p(U)->twBefore,
                                         Route::twBetween(nn(U), V),
                                         n(U)->tw,
                                         U->tw,
                                         n(V)->twAfter);

            deltaCost += d_penalties->timeWarp(uTWS);
        }
        else
        {
            auto const uTWS = TWS::merge(V->twBefore,
                                         n(U)->tw,
                                         U->tw,
                                         Route::twBetween(n(V), p(U)),
                                         nn(U)->twAfter);

            deltaCost += d_penalties->timeWarp(uTWS);
        }

        deltaCost -= d_penalties->timeWarp(U->route->tw);
    }

    return deltaCost;
}
