#include "operators.h"

#include "TimeWindowSegment.h"

bool moveTwoClientsReversed(Node *U, Node *V, Penalties const &penalties)
{
    using TWS = TimeWindowSegment;

    auto const &params = *U->params;

    if (U == n(V) || n(U) == V || n(U)->isDepot)
        return false;

    int const current
        = params.dist(p(U)->client, U->client, n(U)->client, nn(U)->client)
          + params.dist(V->client, n(V)->client);
    int const proposed
        = params.dist(p(U)->client, nn(U)->client)
          + params.dist(V->client, n(U)->client, U->client, n(V)->client);

    int deltaCost = proposed - current;

    if (U->route != V->route)
    {
        if (U->route->isFeasible() && deltaCost >= 0)
            return false;

        auto uTWS = TWS::merge(p(U)->twBefore, nn(U)->twAfter);
        auto vTWS = TWS::merge(V->twBefore, n(U)->tw, U->tw, n(V)->twAfter);
        auto const uDemand = params.clients[U->client].demand;
        auto const xDemand = params.clients[n(U)->client].demand;

        deltaCost += penalties.load(U->route->load - uDemand - xDemand)
                     + penalties.timeWarp(uTWS) - U->route->penalty
                     + penalties.load(V->route->load + uDemand + xDemand)
                     + penalties.timeWarp(vTWS) - V->route->penalty;
    }
    else  // within same route
    {
        if (!U->route->hasTimeWarp() && deltaCost >= 0)
            return false;

        if (U->position < V->position)
        {
            auto const uTWS = TWS::merge(p(U)->twBefore,
                                         Route::twBetween(nn(U), V),
                                         n(U)->tw,
                                         U->tw,
                                         n(V)->twAfter);

            deltaCost += penalties.timeWarp(uTWS);
        }
        else
        {
            auto const uTWS = TWS::merge(V->twBefore,
                                         n(U)->tw,
                                         U->tw,
                                         Route::twBetween(n(V), p(U)),
                                         nn(U)->twAfter);

            deltaCost += penalties.timeWarp(uTWS);
        }

        deltaCost += penalties.load(U->route->load) - U->route->penalty;
    }

    if (deltaCost >= 0)
        return false;

    auto *X = n(U);  // copy since the insert below changes n(U)

    U->insertAfter(V);
    X->insertAfter(V);

    return true;
}
