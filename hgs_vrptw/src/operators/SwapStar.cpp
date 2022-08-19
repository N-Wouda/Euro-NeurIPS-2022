#include "SwapStar.h"

namespace
{
using TWS = TimeWindowSegment;
}

void SwapStar::preprocess(Route *R1, Route *R2)
{
    for (Node *U = n(R1->depot); !U->isDepot(); U = n(U))
    {
        auto &currentOption = cache(R2->idx, U->client);
        currentOption = {};

        // Performs the preprocessing
        // Note: when removing U and adding V to a route, the timewarp
        // penalties may interact, however in most cases it will hold that the
        // reduced timewarp from removing U + added timewarp from adding V will
        // be bigger than the actual delta timewarp such that assuming
        // independence gives a conservative estimate

        auto twData = TWS::merge(p(U)->twBefore, n(U)->twAfter);
        int const deltaRemoval
            = d_params.dist(p(U)->client, n(U)->client)
              - d_params.dist(p(U)->client, U->client, n(U)->client)
              + d_penalties->timeWarp(twData) - d_penalties->timeWarp(R1->tw);

        // Compute additional timewarp we get when inserting U in R2, this
        // may be actually less if we remove U but we ignore this to have a
        // conservative estimate
        twData = TWS::merge(R2->depot->twBefore, U->tw, n(R2->depot)->twAfter);

        int cost = d_params.dist(0, U->client, n(R2->depot)->client)
                   - d_params.dist(0, n(R2->depot)->client)
                   + d_penalties->timeWarp(twData)
                   - d_penalties->timeWarp(R2->tw);

        currentOption.maybeAdd(cost - deltaRemoval, R2->depot);

        for (Node *V = n(R2->depot); !V->isDepot(); V = n(V))
        {
            twData = TWS::merge(V->twBefore, U->tw, n(V)->twAfter);
            int deltaCost = d_params.dist(V->client, U->client, n(V)->client)
                            - d_params.dist(V->client, n(V)->client)
                            + d_penalties->timeWarp(twData)
                            - d_penalties->timeWarp(R2->tw);

            currentOption.maybeAdd(deltaCost - deltaRemoval, V);
        }
    }
}

// Gets the best reinsert point for U in the route of V, assuming V is removed.
// Returns the cost delta.
int SwapStar::getBestInsertPoint(Node *U,
                                 Node *V,
                                 Node *&pos,
                                 SwapStar::ThreeBest const &bestPos)
{
    // Finds the best insertion in the route such that V is not adjacent
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
    int deltaCost = d_params.dist(p(V)->client, U->client, n(V)->client)
                    - d_params.dist(p(V)->client, n(V)->client)
                    + d_penalties->timeWarp(twData)
                    - d_penalties->timeWarp(V->route->tw);

    if (!found || deltaCost < bestCost)
    {
        pos = p(V);
        bestCost = deltaCost;
    }

    return bestCost;
}

void SwapStar::init(Individual const &indiv, Penalties const *penalties)
{
    LocalSearchOperator<Route>::init(indiv, penalties);
    cache.clear();
}

int SwapStar::evaluate(Route *routeU, Route *routeV)
{
    best = {};

    preprocess(routeU, routeV);  // TODO can we cache some of
    preprocess(routeV, routeU);  //   this?

    for (Node *U = n(routeU->depot); !U->isDepot(); U = n(U))
    {
        auto const &bestU = cache(routeV->idx, U->client);

        for (Node *V = n(routeV->depot); !V->isDepot(); V = n(V))
        {
            auto const &bestV = cache(routeU->idx, V->client);

            // We cannot determine impact on time warp without impacting
            // performance too much (cubic vs. currently quadratic)
            int const uDemand = d_params.clients[U->client].demand;
            int const vDemand = d_params.clients[V->client].demand;

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
        = d_params.dist(p(best.U)->client, best.U->client, n(best.U)->client)
          + d_params.dist(p(best.V)->client, best.V->client, n(best.V)->client);

    int const proposed = d_params.dist(best.VAfter->client, best.V->client)
                         + d_params.dist(best.UAfter->client, best.U->client);

    int deltaCost = proposed - current;

    if (best.VAfter == p(best.U))
    {
        // Insert in place of U
        deltaCost += d_params.dist(best.V->client, n(best.U)->client);
    }
    else
    {
        deltaCost
            += d_params.dist(best.V->client, n(best.VAfter)->client)
               + d_params.dist(p(best.U)->client, n(best.U)->client)
               - d_params.dist(best.VAfter->client, n(best.VAfter)->client);
    }

    if (best.UAfter == p(best.V))
    {
        // Insert in place of V
        deltaCost += d_params.dist(best.U->client, n(best.V)->client);
    }
    else
    {
        deltaCost
            += d_params.dist(best.U->client, n(best.UAfter)->client)
               + d_params.dist(p(best.V)->client, n(best.V)->client)
               - d_params.dist(best.UAfter->client, n(best.UAfter)->client);
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

    auto const uDemand = d_params.clients[best.U->client].demand;
    auto const vDemand = d_params.clients[best.V->client].demand;

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
