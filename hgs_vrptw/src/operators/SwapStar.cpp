#include "SwapStar.h"

namespace
{
using TWS = TimeWindowSegment;
}

std::vector<SwapStar::ThreeBest> SwapStar::preprocess(Route *R1, Route *R2)
{
    auto const &params = *R1->params;
    std::vector<ThreeBest> from2to;  // stores insertion points for each node

    for (Node *U = n(R1->depot); !U->isDepot(); U = n(U))
    {
        auto &currentOption = from2to.emplace_back();

        // Performs the preprocessing
        // Note: when removing U and adding V to a route, the timewarp
        // penalties may interact, however in most cases it will hold that the
        // reduced timewarp from removing U + added timewarp from adding V will
        // be bigger than the actual delta timewarp such that assuming
        // independence gives a conservative estimate

        auto twData = TWS::merge(p(U)->twBefore, n(U)->twAfter);
        int const deltaRemoval
            = params.dist(p(U)->client, n(U)->client)
              - params.dist(p(U)->client, U->client, n(U)->client)
              + d_penalties->timeWarp(twData) - d_penalties->timeWarp(R1->tw);

        // Compute additional timewarp we get when inserting U in R2, this
        // may be actually less if we remove U but we ignore this to have a
        // conservative estimate
        twData = TWS::merge(R2->depot->twBefore, U->tw, n(R2->depot)->twAfter);

        int cost = params.dist(0, U->client, n(R2->depot)->client)
                   - params.dist(0, n(R2->depot)->client)
                   + d_penalties->timeWarp(twData)
                   - d_penalties->timeWarp(R2->tw);

        currentOption.maybeAdd(cost - deltaRemoval, R2->depot);

        for (Node *V = n(R2->depot); !V->isDepot(); V = n(V))
        {
            twData = TWS::merge(V->twBefore, U->tw, n(V)->twAfter);
            int deltaCost = params.dist(V->client, U->client, n(V)->client)
                            - params.dist(V->client, n(V)->client)
                            + d_penalties->timeWarp(twData)
                            - d_penalties->timeWarp(R2->tw);

            currentOption.maybeAdd(deltaCost - deltaRemoval, V);
        }
    }

    return from2to;
}

// Gets the best reinsert point for U in the route of V, assuming V is removed.
// Returns the cost delta.
int SwapStar::getBestInsertPoint(Node *U,
                                 Node *V,
                                 Node *&pos,
                                 SwapStar::ThreeBest const &bestPos)
{
    auto const &params = *U->params;

    // Finds the bestPos insertion in the route such that V is not adjacent
    pos = bestPos.locs[0];
    int bestCost = bestPos.costs[0];
    bool found = (pos && pos != V && n(pos) != V);
    if (!found && bestPos.locs[1])
    {
        pos = bestPos.locs[1];
        bestCost = bestPos.costs[1];
        found = (pos != V && n(pos) != V);
        if (!found && bestPos.locs[2])
        {
            pos = bestPos.locs[2];
            bestCost = bestPos.costs[2];
            found = true;
        }
    }

    // Also test inserting in the place of V
    auto const twData = TWS::merge(p(V)->twBefore, U->tw, n(V)->twAfter);
    int deltaCost = params.dist(p(V)->client, U->client, n(V)->client)
                    - params.dist(p(V)->client, n(V)->client)
                    + d_penalties->timeWarp(twData)
                    - d_penalties->timeWarp(V->route->tw);

    if (!found || deltaCost < bestCost)
    {
        pos = p(V);
        bestCost = deltaCost;
    }

    return bestCost;
}

int SwapStar::test(Route *routeU, Route *routeV)
{
    best = {};

    auto const u2v = preprocess(routeU, routeV);  // TODO can we cache some of
    auto const v2u = preprocess(routeV, routeU);  //   this?

    auto const &params = *routeU->params;

    for (Node *U = n(routeU->depot); !U->isDepot(); U = n(U))
    {
        auto const &bestU = u2v[U->position - 1];

        for (Node *V = n(routeV->depot); !V->isDepot(); V = n(V))
        {
            auto const &bestV = v2u[V->position - 1];

            // We cannot determine impact on time warp without impacting
            // performance too much (cubic vs. currently quadratic)
            int const uDemand = params.clients[U->client].demand;
            int const vDemand = params.clients[V->client].demand;

            int const loadPenU
                = d_penalties->load(routeU->load + vDemand - uDemand);
            int const loadPenV
                = d_penalties->load(routeV->load + uDemand - vDemand);

            int const deltaLoadPen = loadPenU + loadPenV
                                     - d_penalties->load(routeU->load)
                                     - d_penalties->load(routeV->load);
            int const bestOption = bestU.costs[0] + bestV.costs[0];

            if (deltaLoadPen + bestOption <= 0)  // quick filter on many moves
            {
                Node *UAfter;
                Node *VAfter;

                int extraV = getBestInsertPoint(U, V, UAfter, bestU);
                int extraU = getBestInsertPoint(V, U, VAfter, bestV);
                int cost = deltaLoadPen + bestOption + extraU + extraV;

                if (cost < best.cost)
                {
                    best.cost = cost;

                    best.U = U;
                    best.UAfter = UAfter;

                    best.V = V;
                    best.VAfter = VAfter;
                }
            }
        }
    }

    if (!best.UAfter || !best.VAfter)
        return 0;

    // Compute actual cost including TimeWarp penalty
    int const current
        = params.dist(p(best.U)->client, best.U->client, n(best.U)->client)
          + params.dist(p(best.V)->client, best.V->client, n(best.V)->client);

    int const proposed = params.dist(best.VAfter->client, best.V->client)
                         + params.dist(best.UAfter->client, best.U->client);

    int deltaCost = proposed - current;

    if (best.VAfter == p(best.U))
    {
        // Insert in place of U
        deltaCost += params.dist(best.V->client, n(best.U)->client);
    }
    else
    {
        deltaCost += params.dist(best.V->client, n(best.VAfter)->client)
                     + params.dist(p(best.U)->client, n(best.U)->client)
                     - params.dist(best.VAfter->client, n(best.VAfter)->client);
    }

    if (best.UAfter == p(best.V))
    {
        // Insert in place of V
        deltaCost += params.dist(best.U->client, n(best.V)->client);
    }
    else
    {
        deltaCost += params.dist(best.U->client, n(best.UAfter)->client)
                     + params.dist(p(best.V)->client, n(best.V)->client)
                     - params.dist(best.UAfter->client, n(best.UAfter)->client);
    }

    // It is not possible to have UAfter == V or VAfter == U, so the positions
    // are always strictly different
    if (best.VAfter->position + 1 == best.U->position)
    {
        // Special case
        auto uTWS
            = TWS::merge(best.VAfter->twBefore, best.V->tw, n(best.U)->twAfter);

        deltaCost += d_penalties->timeWarp(uTWS);
    }
    else if (best.VAfter->position < best.U->position)
    {
        auto uTWS = TWS::merge(best.VAfter->twBefore,
                               best.V->tw,
                               Route::twBetween(n(best.VAfter), p(best.U)),
                               n(best.U)->twAfter);

        deltaCost += d_penalties->timeWarp(uTWS);
    }
    else
    {
        auto uTWS = TWS::merge(p(best.U)->twBefore,
                               Route::twBetween(n(best.U), best.VAfter),
                               best.V->tw,
                               n(best.VAfter)->twAfter);

        deltaCost += d_penalties->timeWarp(uTWS);
    }

    if (best.UAfter->position + 1 == best.V->position)
    {
        // Special case
        auto vTWS
            = TWS::merge(best.UAfter->twBefore, best.U->tw, n(best.V)->twAfter);

        deltaCost += d_penalties->timeWarp(vTWS);
    }
    else if (best.UAfter->position < best.V->position)
    {
        auto vTWS = TWS::merge(best.UAfter->twBefore,
                               best.U->tw,
                               Route::twBetween(n(best.UAfter), p(best.V)),
                               n(best.V)->twAfter);

        deltaCost += d_penalties->timeWarp(vTWS);
    }
    else
    {
        auto vTWS = TWS::merge(p(best.V)->twBefore,
                               Route::twBetween(n(best.V), best.UAfter),
                               best.U->tw,
                               n(best.UAfter)->twAfter);

        deltaCost += d_penalties->timeWarp(vTWS);
    }

    deltaCost -= d_penalties->timeWarp(routeU->tw);
    deltaCost -= d_penalties->timeWarp(routeV->tw);

    auto const uDemand = params.clients[best.U->client].demand;
    auto const vDemand = params.clients[best.V->client].demand;

    deltaCost += d_penalties->load(routeU->load - uDemand + vDemand);
    deltaCost -= d_penalties->load(routeU->load);

    deltaCost += d_penalties->load(routeV->load + uDemand - vDemand);
    deltaCost -= d_penalties->load(routeV->load);

    return deltaCost;
}

void SwapStar::apply(Route *U, Route *V)
{
    if (best.U && best.UAfter && best.V && best.VAfter)
    {
        best.U->insertAfter(best.UAfter);
        best.V->insertAfter(best.VAfter);
    }
}
