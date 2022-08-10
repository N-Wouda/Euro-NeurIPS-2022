#include "TwoOpt.h"

#include "Route.h"
#include "TimeWindowSegment.h"

namespace
{
using TWS = TimeWindowSegment;
}

bool TwoOpt::withinRouteTest(Node *U, Node *V)
{
    auto const &params = *U->params;

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

    auto tws = U->twBefore;
    auto *itRoute = V;
    while (itRoute != U)
    {
        tws = TWS::merge(tws, itRoute->tw);
        itRoute = p(itRoute);
    }

    tws = TWS::merge(tws, n(V)->twAfter);

    deltaCost += d_penalties->timeWarp(tws);
    deltaCost -= d_penalties->timeWarp(U->route->tw);

    return deltaCost < 0;
}

bool TwoOpt::betweenRouteTest(Node *U, Node *V)
{
    auto const &params = *U->params;

    int const current = params.dist(U->client, n(U)->client)
                        + params.dist(V->client, n(V)->client);
    int const proposed = params.dist(U->client, n(V)->client)
                         + params.dist(V->client, n(U)->client);

    int deltaCost = proposed - current;

    if (U->route->isFeasible() && V->route->isFeasible() && deltaCost >= 0)
        return false;

    auto const uTWS = TWS::merge(U->twBefore, n(V)->twAfter);

    deltaCost += d_penalties->timeWarp(uTWS);
    deltaCost -= d_penalties->timeWarp(U->route->tw);

    auto const vTWS = TWS::merge(V->twBefore, n(U)->twAfter);

    deltaCost += d_penalties->timeWarp(vTWS);
    deltaCost -= d_penalties->timeWarp(V->route->tw);

    int const deltaLoad = U->cumulatedLoad - V->cumulatedLoad;

    deltaCost += d_penalties->load(U->route->load - deltaLoad);
    deltaCost -= d_penalties->load(U->route->load);

    deltaCost += d_penalties->load(V->route->load + deltaLoad);
    deltaCost -= d_penalties->load(V->route->load);

    return deltaCost < 0;
}

void TwoOpt::withinRouteApply(Node *U, Node *V)
{
    auto *itRoute = V;
    auto *insertionPoint = U;
    auto *currNext = n(U);

    while (itRoute != currNext)  // No need to move x, we pivot around it
    {
        auto *current = itRoute;
        itRoute = p(itRoute);
        current->insertAfter(insertionPoint);
        insertionPoint = current;
    }
}

void TwoOpt::betweenRouteApply(Node *U, Node *V)
{
    auto *itRouteU = n(U);
    auto *itRouteV = n(V);

    auto *insertLocation = U;
    while (!itRouteV->isDepot())
    {
        auto *node = itRouteV;
        itRouteV = n(itRouteV);
        node->insertAfter(insertLocation);
        insertLocation = node;
    }

    insertLocation = V;
    while (!itRouteU->isDepot())
    {
        auto *node = itRouteU;
        itRouteU = n(itRouteU);
        node->insertAfter(insertLocation);
        insertLocation = node;
    }
}

bool TwoOpt::test(Node *U, Node *V)
{
    if (U->route->idx > V->route->idx)  // will be tackled in a later iteration
        return false;                   // - no need to process here already

    return U->route->idx == V->route->idx ? withinRouteTest(U, V)
                                          : betweenRouteTest(U, V);
}

void TwoOpt::apply(Node *U, Node *V)
{
    if (U->route->idx == V->route->idx)
        withinRouteApply(U, V);
    else
        betweenRouteApply(U, V);
}
