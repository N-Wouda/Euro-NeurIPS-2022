#include "operators.h"

#include <array>

namespace
{
// Structure used in SWAP* to remember the three best insertion positions of
// a client in a given route
struct ThreeBest
{
    std::array<int, 3> bestCost = {INT_MAX, INT_MAX, INT_MAX};
    std::array<Node *, 3> bestLocation = {nullptr, nullptr, nullptr};

    void add(int costInsert, Node *placeInsert)
    {
        if (costInsert >= bestCost[2])
            return;

        if (costInsert >= bestCost[1])
        {
            bestCost[2] = costInsert;
            bestLocation[2] = placeInsert;
        }
        else if (costInsert >= bestCost[0])
        {
            bestCost[2] = bestCost[1];
            bestLocation[2] = bestLocation[1];
            bestCost[1] = costInsert;
            bestLocation[1] = placeInsert;
        }
        else
        {
            bestCost[2] = bestCost[1];
            bestLocation[2] = bestLocation[1];
            bestCost[1] = bestCost[0];
            bestLocation[1] = bestLocation[0];
            bestCost[0] = costInsert;
            bestLocation[0] = placeInsert;
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
}  // namespace

bool swapStar(Route *routeU, Route *routeV, Penalties const &penalties)
{
    // Preprocessing phase
    std::vector<ThreeBest> u2v;
    for (auto *U = routeU->depot->next; !U->isDepot; U = U->next)
    {
        auto &best = u2v.emplace_back();

        for (auto *V = routeV->depot->next; !V->isDepot; V = V->next)
        {
            int deltaCost = operators::singleMoveCost(U, V, penalties);
            best.add(deltaCost, V);
        }

        int deltaCost = operators::singleMoveCost(U, routeV->depot, penalties);
        best.add(deltaCost, routeV->depot);
    }

    std::vector<ThreeBest> v2u;
    for (auto *V = routeV->depot->next; !V->isDepot; V = V->next)
    {
        auto &best = v2u.emplace_back();

        for (auto *U = routeU->depot->next; !U->isDepot; U = U->next)
        {
            int deltaCost = operators::singleMoveCost(V, U, penalties);
            best.add(deltaCost, U);
        }

        int deltaCost = operators::singleMoveCost(V, routeU->depot, penalties);
        best.add(deltaCost, routeU->depot);
    }

    // Search phase
    SwapStarMove best;

    for (auto *U = routeU->depot->next; !U->isDepot; U = U->next)
        for (auto *V = routeV->depot->next; !V->isDepot; V = V->next)
        {
            auto const &UBest = u2v[U->position - 1];
            auto const &VBest = v2u[V->position - 1];

            // TODO use other indices as well
            auto const deltaCostStar = UBest.bestCost[0] + VBest.bestCost[0];
            auto const deltaCostSwap = operators::twoSwapCost(U, V, penalties);

            if (deltaCostStar < best.deltaCost && deltaCostStar < deltaCostSwap)
            {
                best.deltaCost = deltaCostStar;
                best.U = U;
                best.UAfter = UBest.bestLocation[0];

                best.V = V;
                best.VAfter = VBest.bestLocation[0];
            }

            if (deltaCostSwap < best.deltaCost)
            {
                best.deltaCost = deltaCostSwap;
                best.U = U;
                best.UAfter = UBest.bestLocation[0];

                best.V = V;
                best.VAfter = VBest.bestLocation[0];
            }
        }

    auto &[deltaCost, U, V, UAfter, VAfter] = best;

    if (deltaCost < 0)
    {
        U->insertAfter(UAfter);
        V->insertAfter(VAfter);
    }

    return deltaCost < 0;
}
