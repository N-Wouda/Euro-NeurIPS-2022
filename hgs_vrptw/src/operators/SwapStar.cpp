#include "SwapStar.h"

namespace
{
using TWS = TimeWindowSegment;
}

void SwapStar::updateRemovalCosts(Route *R1)
{
    auto const currTimeWarp = d_penalties->timeWarp(R1->tw);

    for (Node *U = n(R1->depot); !U->isDepot(); U = n(U))
    {
        auto twData = TWS::merge(p(U)->twBefore, n(U)->twAfter);
        removalCosts(R1->idx, U->client)
            = d_params.dist(p(U)->client, n(U)->client)
              - d_params.dist(p(U)->client, U->client, n(U)->client)
              + d_penalties->timeWarp(twData) - currTimeWarp;
    }
}

void SwapStar::updateInsertionCost(Route *R, Node *U)
{
    auto &insertPositions = cache(R->idx, U->client);

    insertPositions = {};
    insertPositions.shouldUpdate = false;

    // Insert cost of U just after the depot (0 -> U -> ...)
    auto twData = TWS::merge(R->depot->twBefore, U->tw, n(R->depot)->twAfter);
    int cost = d_params.dist(0, U->client, n(R->depot)->client)
               - d_params.dist(0, n(R->depot)->client)
               + d_penalties->timeWarp(twData) - d_penalties->timeWarp(R->tw);

    insertPositions.maybeAdd(cost, R->depot);

    for (Node *V = n(R->depot); !V->isDepot(); V = n(V))
    {
        // Insert cost of U just after V (V -> U -> ...)
        twData = TWS::merge(V->twBefore, U->tw, n(V)->twAfter);
        int deltaCost = d_params.dist(V->client, U->client, n(V)->client)
                        - d_params.dist(V->client, n(V)->client)
                        + d_penalties->timeWarp(twData)
                        - d_penalties->timeWarp(R->tw);

        insertPositions.maybeAdd(deltaCost, V);
    }
}

std::pair<int, Node *> SwapStar::getBestInsertPoint(Node *U, Node *V)
{
    auto &best_ = cache(V->route->idx, U->client);

    if (best_.shouldUpdate)  // then we first update the insert positions
        updateInsertionCost(V->route, U);

    for (size_t idx = 0; idx != 3; ++idx)  // only OK if V is not adjacent
        if (best_.locs[idx] && best_.locs[idx] != V && n(best_.locs[idx]) != V)
            return std::make_pair(best_.costs[idx], best_.locs[idx]);

    // As a fallback option, we consider inserting in the place of V
    auto const twData = TWS::merge(p(V)->twBefore, U->tw, n(V)->twAfter);
    int deltaCost = d_params.dist(p(V)->client, U->client, n(V)->client)
                    - d_params.dist(p(V)->client, n(V)->client)
                    + d_penalties->timeWarp(twData)
                    - d_penalties->timeWarp(V->route->tw);

    return std::make_pair(deltaCost, p(V));
}

void SwapStar::init(Individual const &indiv, Penalties const *penalties)
{
    LocalSearchOperator<Route>::init(indiv, penalties);

    cache.clear();
    updated = std::vector<bool>(d_params.nbVehicles);
    removalCosts.clear();
}

int SwapStar::evaluate(Route *routeU, Route *routeV)
{
    best = {};

    if (updated[routeV->idx])
    {
        updateRemovalCosts(routeV);
        updated[routeV->idx] = false;
    }

    if (updated[routeU->idx])
    {
        updateRemovalCosts(routeU);
        updated[routeV->idx] = false;
    }

    for (Node *U = n(routeU->depot); !U->isDepot(); U = n(U))
        for (Node *V = n(routeV->depot); !V->isDepot(); V = n(V))
        {
            // We cannot determine time warp delta without impacting
            // performance too much (cubic vs. currently quadratic)
            int deltaCost = 0;

            int const uDemand = d_params.clients[U->client].demand;
            int const vDemand = d_params.clients[V->client].demand;
            int const loadDiff = uDemand - vDemand;

            deltaCost += d_penalties->load(routeU->load - loadDiff);
            deltaCost -= d_penalties->load(routeU->load);

            deltaCost += d_penalties->load(routeV->load + loadDiff);
            deltaCost -= d_penalties->load(routeV->load);

            deltaCost += removalCosts(routeU->idx, U->client);
            deltaCost += removalCosts(routeV->idx, V->client);

            if (deltaCost < 0)  // an early filter on many moves, before doing
            {                   // costly work determining insertion points
                auto [extraV, UAfter] = getBestInsertPoint(U, V);
                auto [extraU, VAfter] = getBestInsertPoint(V, U);
                deltaCost += extraU + extraV;

                if (deltaCost < best.cost)
                {
                    best.cost = deltaCost;

                    best.U = U;
                    best.UAfter = UAfter;

                    best.V = V;
                    best.VAfter = VAfter;
                }
            }
        }

    if (!best.UAfter || !best.VAfter)
        return 0;

    // Compute actual cost including time warp penalty
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

void SwapStar::update(Route *U, size_t locU)
{
    updated[U->idx] = true;

    for (int idx = 1; idx != d_params.nbClients + 1; ++idx)
        cache(U->idx, idx).shouldUpdate = true;
}

void SwapStar::update(Route *U, Route *V, size_t locU, size_t locV)
{
    update(U, locU);
    update(V, locV);
}

void SwapStar::apply(Route *U, Route *V)
{
    if (best.U && best.UAfter && best.V && best.VAfter)
    {
        best.U->insertAfter(best.UAfter);
        best.V->insertAfter(best.VAfter);
    }
}
