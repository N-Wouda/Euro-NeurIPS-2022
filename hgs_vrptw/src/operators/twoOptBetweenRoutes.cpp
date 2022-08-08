#include "operators.h"

#include "TimeWindowSegment.h"

bool twoOptBetweenRoutes(Node *U, Node *V, Penalties const &penalties)
{
    using TWS = TimeWindowSegment;

    auto const &params = *U->params;

    if (U->route->idx >= V->route->idx)
        return false;

    int const current = params.dist(U->client, n(U)->client)
                        + params.dist(V->client, n(V)->client);
    int const proposed = params.dist(U->client, n(V)->client)
                         + params.dist(V->client, n(U)->client);

    int deltaCost = proposed - current;

    if (U->route->isFeasible() && V->route->isFeasible() && deltaCost >= 0)
        return false;

    auto const uTWS = TWS::merge(U->twBefore, n(V)->twAfter);
    auto const vTWS = TWS::merge(V->twBefore, n(U)->twAfter);

    int const deltaLoad = U->cumulatedLoad - V->cumulatedLoad;

    deltaCost += penalties.load(V->route->load + deltaLoad)
                 + penalties.timeWarp(uTWS) - U->route->penalty
                 + penalties.load(U->route->load - deltaLoad)
                 + penalties.timeWarp(vTWS) - V->route->penalty;

    if (deltaCost >= 0)
        return false;

    auto *itRouteU = n(U);
    auto *itRouteV = n(V);

    auto *insertLocation = U;
    while (!itRouteV->isDepot())
    {
        auto *current = itRouteV;
        itRouteV = n(itRouteV);
        current->insertAfter(insertLocation);
        insertLocation = current;
    }

    insertLocation = V;
    while (!itRouteU->isDepot())
    {
        auto *current = itRouteU;
        itRouteU = n(itRouteU);
        current->insertAfter(insertLocation);
        insertLocation = current;
    }

    return true;
}
