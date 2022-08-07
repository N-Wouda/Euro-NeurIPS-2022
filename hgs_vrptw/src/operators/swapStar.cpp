#include "operators.h"

#include <array>
#include <cassert>

namespace
{
// Stores the three best insertion positions of a client in a given route
struct ThreeBest
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

struct SwapStarMove
{
    int deltaCost = INT_MAX;
    Node *U = nullptr;
    Node *V = nullptr;
    Node *UAfter = nullptr;
    Node *VAfter = nullptr;
};

std::vector<ThreeBest>
preprocess(Route *from, Route *to, Penalties const &penalties)
{
    std::vector<ThreeBest> from2to;  // stores insertion points for each node
    for (auto *U = from->depot->next; !U->isDepot; U = U->next)
    {
        auto &best = from2to.emplace_back();

        for (auto *V = to->depot->next; !V->isDepot; V = V->next)
        {
            int deltaCost = operators::singleMoveCost(U, V, penalties);
            best.maybeAdd(deltaCost, V);
        }

        int deltaCost = operators::singleMoveCost(U, to->depot, penalties);
        best.maybeAdd(deltaCost, to->depot);
    }

    return from2to;
}
}  // namespace

bool swapStar(Route *routeU, Route *routeV, Penalties const &penalties)
{
    auto const u2v = preprocess(routeU, routeV, penalties);
    auto const v2u = preprocess(routeV, routeU, penalties);

    SwapStarMove best;
    for (auto *U = routeU->depot->next; !U->isDepot; U = U->next)
        for (auto *V = routeV->depot->next; !V->isDepot; V = V->next)
        {
            // First evaluate a direct swap of U and V
            int const deltaSwap = operators::twoSwapCost(U, V, penalties);

            if (deltaSwap < best.deltaCost)
            {
                best.deltaCost = deltaSwap;
                best.U = U;
                best.UAfter = V;

                best.V = V;
                best.VAfter = U;
            }

            // Next evaluate swaps within the entire routes U and V using the
            // three best insert locations.
            auto const &UBest = u2v[U->position - 1];
            auto const &VBest = v2u[V->position - 1];

            for (size_t idx1 = 0; idx1 != 3; ++idx1)
                for (size_t idx2 = 0; idx2 != 3; ++idx2)
                {
                    if (UBest.costs[idx1] == INT_MAX      // unchanged, so must
                        || VBest.costs[idx2] == INT_MAX)  // not be evaluated
                        continue;

                    int const deltaStar = UBest.costs[idx1] + VBest.costs[idx2];

                    if (deltaStar < best.deltaCost)
                    {
                        best.deltaCost = deltaStar;
                        best.U = U;
                        best.UAfter = UBest.locs[idx1];

                        best.V = V;
                        best.VAfter = VBest.locs[idx2];
                    }
                }
        }

    auto const &[deltaCost, U, V, UAfter, VAfter] = best;

    if (deltaCost < 0)
    {
        assert(U && V && UAfter && VAfter);

        U->insertAfter(UAfter);
        V->insertAfter(VAfter);
    }

    return deltaCost < 0;
}
