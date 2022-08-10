#include "RelocateStar.h"

#include "TimeWindowSegment.h"

int RelocateStar::singleMoveCost(Node *U, Node *V) const
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

        deltaCost += d_penalties->timeWarp(uTWS);
        deltaCost -= d_penalties->timeWarp(U->route->tw);

        auto const uDemand = params.clients[U->client].demand;

        deltaCost += d_penalties->load(U->route->load - uDemand);
        deltaCost -= d_penalties->load(U->route->load);

        if (deltaCost >= 0)    // if delta cost of just U's route is not enough
            return deltaCost;  // even without V, the move will never be good

        deltaCost += d_penalties->load(V->route->load + uDemand);
        deltaCost -= d_penalties->load(V->route->load);

        auto const vTWS = TWS::merge(V->twBefore, U->tw, n(V)->twAfter);

        deltaCost += d_penalties->timeWarp(vTWS);
        deltaCost -= d_penalties->timeWarp(V->route->tw);
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

            deltaCost += d_penalties->timeWarp(uTWS);
        }
        else
        {
            auto const uTWS = TWS::merge(V->twBefore,
                                         U->tw,
                                         Route::twBetween(n(V), p(U)),
                                         n(U)->twAfter);

            deltaCost += d_penalties->timeWarp(uTWS);
        }

        deltaCost -= d_penalties->timeWarp(U->route->tw);
    }

    return deltaCost;
}

bool RelocateStar::test(Route *U, Route *V)
{
    bestCost = 0;
    insertionPoint = nullptr;
    nodeToInsert = nullptr;

    auto eval = [&](auto *nodeU, auto *nodeV) {
        int const deltaCost = singleMoveCost(nodeU, nodeV);

        if (deltaCost < bestCost)
        {
            bestCost = deltaCost;
            insertionPoint = nodeV;
            nodeToInsert = nodeU;
        }
    };

    for (auto *nodeU = n(U->depot); !nodeU->isDepot(); nodeU = n(nodeU))
    {
        eval(nodeU, V->depot);

        for (auto *nodeV = n(V->depot); !nodeV->isDepot(); nodeV = n(nodeV))
            eval(nodeU, nodeV);
    }

    return bestCost < 0 && insertionPoint && nodeToInsert;
}
