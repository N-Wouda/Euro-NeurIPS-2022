#include "operators.h"

#include <array>

namespace
{
using TWS = TimeWindowSegment;

struct ThreeBest  // stores three best SWAP* insertion points
{
    std::array<int, 3> costs = {INT_MAX, INT_MAX, INT_MAX};
    std::array<Node *, 3> locs = {nullptr, nullptr, nullptr};

    void maybeAdd(int costInsert, Node *placeInsert)
    {
        if (costInsert >= costs[2])
            return;

        if (costInsert >= costs[1])
        {
            costs[2] = costInsert;
            locs[2] = placeInsert;
        }
        else if (costInsert >= costs[0])
        {
            costs[2] = costs[1];
            locs[2] = locs[1];
            costs[1] = costInsert;
            locs[1] = placeInsert;
        }
        else
        {
            costs[2] = costs[1];
            locs[2] = locs[1];
            costs[1] = costs[0];
            locs[1] = locs[0];
            costs[0] = costInsert;
            locs[0] = placeInsert;
        }
    }
};

struct BestMove  // tracks the best SWAP* move
{
    int moveCost = INT_MAX;
    int loadPenU = INT_MAX;
    int loadPenV = INT_MAX;
    Node *U = nullptr;
    Node *UAfter = nullptr;
    Node *V = nullptr;
    Node *VAfter = nullptr;
};

std::vector<ThreeBest>
preprocess(Route *R1, Route *R2, Penalties const &penalties)
{
    auto const &params = *R1->params;
    std::vector<ThreeBest> from2to;  // stores insertion points for each node

    for (Node *U = R1->depot->next; !U->isDepot; U = U->next)
    {
        auto &currentOption = from2to.emplace_back();

        // Performs the preprocessing
        // Note: when removing U and adding V to a route, the timewarp penalties
        // may interact, however in most cases it will hold that the reduced
        // timewarp from removing U + added timewarp from adding V will be
        // bigger than the actual delta timewarp such that assuming independence
        // gives a conservative estimate

        auto twData = TWS::merge(U->prev->twBefore, U->next->twAfter);
        int const deltaRemovalTW = params.dist(U->prev->client, U->next->client)
                                   - params.dist(U->prev->client, U->client)
                                   - params.dist(U->client, U->next->client)
                                   + penalties.timeWarp(twData)
                                   - penalties.timeWarp(R1->twData);

        // Compute additional timewarp we get when inserting U in R2, this
        // may be actually less if we remove U but we ignore this to have a
        // conservative estimate
        twData
            = TWS::merge(R2->depot->twBefore, U->tw, R2->depot->next->twAfter);

        int cost = params.dist(0, U->client)
                   + params.dist(U->client, R2->depot->next->client)
                   - params.dist(0, R2->depot->next->client)
                   + penalties.timeWarp(twData)
                   - penalties.timeWarp(R2->twData);

        currentOption.maybeAdd(cost - deltaRemovalTW, R2->depot);

        for (Node *V = R2->depot->next; !V->isDepot; V = V->next)
        {
            twData = TWS::merge(V->twBefore, U->tw, V->next->twAfter);

            int deltaCost = params.dist(V->client, U->client)
                            + params.dist(U->client, V->next->client)
                            - params.dist(V->client, V->next->client)
                            + penalties.timeWarp(twData)
                            - penalties.timeWarp(R2->twData);

            currentOption.maybeAdd(deltaCost - deltaRemovalTW, V);
        }
    }

    return from2to;
}

// Gets the best reinsert point for U in the route of V, assuming V is removed.
// Returns the cost delta.
int getBestInsertPoint(Node *U,
                       Node *V,
                       Node *&pos,
                       ThreeBest const &best,
                       Penalties const &penalties)
{
    auto const &params = *U->params;

    // Finds the best insertion in the route such that V is not next or pred
    // (can only belong to the top three locations)
    pos = best.locs[0];
    int bestCost = best.costs[0];
    bool found = (pos && pos != V && pos->next != V);
    if (!found && best.locs[1])
    {
        pos = best.locs[1];
        bestCost = best.costs[1];
        found = (pos != V && pos->next != V);
        if (!found && best.locs[2])
        {
            pos = best.locs[2];
            bestCost = best.costs[2];
            found = true;
        }
    }

    // Compute insertion in the place of V
    auto const twData = TWS::merge(V->prev->twBefore, U->tw, V->next->twAfter);
    int deltaCost = params.dist(V->prev->client, U->client)
                    + params.dist(U->client, V->next->client)
                    - params.dist(V->prev->client, V->next->client)
                    + penalties.timeWarp(twData)
                    - penalties.timeWarp(V->route->twData);

    if (!found || deltaCost < bestCost)
    {
        pos = V->prev;
        bestCost = deltaCost;
    }

    return bestCost;
}
}  // namespace

bool swapStar(Route *routeU, Route *routeV, Penalties const &penalties)
{
    auto const u2v = preprocess(routeU, routeV, penalties);
    auto const v2u = preprocess(routeV, routeU, penalties);

    auto const &params = *routeU->params;
    BestMove bestMove;

    // Evaluating the moves
    for (Node *U = routeU->depot->next; !U->isDepot; U = U->next)
    {
        auto const &bestU = u2v[U->position - 1];

        for (Node *V = routeV->depot->next; !V->isDepot; V = V->next)
        {
            auto const &bestV = v2u[V->position - 1];

            // We cannot determine impact on timewarp without adding too much
            // complexity (O(n^3) instead of O(n^2))
            int const loadPenU
                = penalties.load(routeU->load + params.clients[V->client].demand
                                 - params.clients[U->client].demand);
            int const loadPenV
                = penalties.load(routeV->load + params.clients[U->client].demand
                                 - params.clients[V->client].demand);
            int const deltaLoadPen = loadPenU + loadPenV
                                     - penalties.load(routeU->load)
                                     - penalties.load(routeV->load);
            int const bestOption = bestU.costs[0] + bestV.costs[0];

            // Quick filter: possibly early elimination of many SWAP* due to the
            // capacity constraints/penalties and bounds on insertion costs
            if (deltaLoadPen + bestOption <= 0)
            {
                Node *UAfter;
                Node *VAfter;

                // Evaluate reinsertion of U and V if the other is removed.
                int extraV = getBestInsertPoint(U, V, UAfter, bestU, penalties);
                int extraU = getBestInsertPoint(V, U, VAfter, bestV, penalties);
                int moveCost = deltaLoadPen + bestOption + extraU + extraV;

                if (moveCost < bestMove.moveCost)
                {
                    bestMove.moveCost = moveCost;
                    bestMove.loadPenU = loadPenU;
                    bestMove.loadPenV = loadPenV;

                    bestMove.U = U;
                    bestMove.UAfter = UAfter;

                    bestMove.V = V;
                    bestMove.VAfter = VAfter;
                }
            }
        }
    }

    if (!bestMove.UAfter || !bestMove.VAfter)
        return false;

    // Compute actual cost including TimeWarp penalty
    int costSuppU = params.dist(bestMove.VAfter->client, bestMove.V->client)
                    - params.dist(bestMove.U->prev->client, bestMove.U->client)
                    - params.dist(bestMove.U->client, bestMove.U->next->client);
    int costSuppV = params.dist(bestMove.UAfter->client, bestMove.U->client)
                    - params.dist(bestMove.V->prev->client, bestMove.V->client)
                    - params.dist(bestMove.V->client, bestMove.V->next->client);

    if (bestMove.VAfter == bestMove.U->prev)
    {
        // Insert in place of U
        costSuppU += params.dist(bestMove.V->client, bestMove.U->next->client);
    }
    else
    {
        costSuppU
            += params.dist(bestMove.V->client, bestMove.VAfter->next->client)
               + params.dist(bestMove.U->prev->client, bestMove.U->next->client)
               - params.dist(bestMove.VAfter->client,
                             bestMove.VAfter->next->client);
    }

    if (bestMove.UAfter == bestMove.V->prev)
    {
        // Insert in place of V
        costSuppV += params.dist(bestMove.U->client, bestMove.V->next->client);
    }
    else
    {
        costSuppV
            += params.dist(bestMove.U->client, bestMove.UAfter->next->client)
               + params.dist(bestMove.V->prev->client, bestMove.V->next->client)
               - params.dist(bestMove.UAfter->client,
                             bestMove.UAfter->next->client);
    }

    // It is not possible to have UAfter == V or VAfter == U, so the positions
    // are always strictly different
    if (bestMove.VAfter->position == bestMove.U->position - 1)
    {
        // Special case
        auto const routeUTwData = TWS::merge(bestMove.VAfter->twBefore,
                                             bestMove.V->tw,
                                             bestMove.U->next->twAfter);

        costSuppU += penalties.timeWarp(routeUTwData);
    }
    else if (bestMove.VAfter->position < bestMove.U->position)
    {
        auto const routeUTwData
            = TWS::merge(bestMove.VAfter->twBefore,
                         bestMove.V->tw,
                         bestMove.U->route->twBetween(bestMove.VAfter->next,
                                                      bestMove.U->prev),
                         bestMove.U->next->twAfter);

        costSuppU += penalties.timeWarp(routeUTwData);
    }
    else
    {
        auto const routeUTwData = TWS::merge(
            bestMove.U->prev->twBefore,
            bestMove.U->route->twBetween(bestMove.U->next, bestMove.VAfter),
            bestMove.V->tw,
            bestMove.VAfter->next->twAfter);

        costSuppU += penalties.timeWarp(routeUTwData);
    }

    if (bestMove.UAfter->position == bestMove.V->position - 1)
    {
        // Special case
        auto const routeVTwData = TWS::merge(bestMove.UAfter->twBefore,
                                             bestMove.U->tw,
                                             bestMove.V->next->twAfter);

        costSuppV += penalties.timeWarp(routeVTwData);
    }
    else if (bestMove.UAfter->position < bestMove.V->position)
    {
        auto const routeVTwData
            = TWS::merge(bestMove.UAfter->twBefore,
                         bestMove.U->tw,
                         bestMove.V->route->twBetween(bestMove.UAfter->next,
                                                      bestMove.V->prev),
                         bestMove.V->next->twAfter);

        costSuppV += penalties.timeWarp(routeVTwData);
    }
    else
    {
        auto const routeVTwData = TWS::merge(
            bestMove.V->prev->twBefore,
            bestMove.V->route->twBetween(bestMove.V->next, bestMove.UAfter),
            bestMove.U->tw,
            bestMove.UAfter->next->twAfter);

        costSuppV += penalties.timeWarp(routeVTwData);
    }

    costSuppU += bestMove.loadPenU - routeU->penalty;
    costSuppV += bestMove.loadPenV - routeV->penalty;

    if (costSuppU + costSuppV >= 0)
        return false;

    bestMove.U->insertAfter(bestMove.UAfter);
    bestMove.V->insertAfter(bestMove.VAfter);

    return true;
}
