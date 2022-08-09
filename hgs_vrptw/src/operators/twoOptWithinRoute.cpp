#include "operators.h"

#include "TimeWindowSegment.h"

bool twoOptWithinRoute(Node *U, Node *V, Penalties const &penalties)
{
    using TWS = TimeWindowSegment;

    auto const &params = *U->params;

    if (U->route != V->route)
        return false;

    if (U->position + 1 >= V->position)
        return false;

    int deltaCost = params.dist(U->client, V->client)
                    + params.dist(n(U)->client, n(V)->client)
                    + V->cumulatedReversalDistance
                    - params.dist(U->client, n(U)->client)
                    - params.dist(V->client, n(V)->client)
                    - n(U)->cumulatedReversalDistance;

    if (!U->route->hasTimeWarp() && deltaCost >= 0)
        return false;

    auto uTWS = U->twBefore;
    auto *itRoute = V;
    while (itRoute != U)
    {
        uTWS = TWS::merge(uTWS, itRoute->tw);
        itRoute = p(itRoute);
    }

    uTWS = TWS::merge(uTWS, n(V)->twAfter);

    deltaCost += penalties.timeWarp(uTWS) - penalties.timeWarp(U->route->tw);

    if (deltaCost >= 0)
        return false;

    itRoute = V;
    auto *insertionPoint = U;
    auto *currNext = n(U);

    while (itRoute != currNext)  // No need to move x, we pivot around it
    {
        auto *current = itRoute;
        itRoute = p(itRoute);
        current->insertAfter(insertionPoint);
        insertionPoint = current;
    }

    return true;
}
