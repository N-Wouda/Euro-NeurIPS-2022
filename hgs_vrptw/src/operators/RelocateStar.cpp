#include "RelocateStar.h"

int RelocateStar::evaluate(Route *U, Route *V)
{
    moves.clear();
    int bestCost = 0;

    auto eval = [&](Node *N1, Node *N2)
    {
        int const deltaCost = relocate.evaluate(N1, N2);

        if (deltaCost < 0)
        {
            bestCost = deltaCost < bestCost ? deltaCost : bestCost;
            moves.emplace_back(deltaCost, N1, N2);
        }
    };

    for (auto *nodeU = n(U->depot); !nodeU->isDepot(); nodeU = n(nodeU))
    {
        eval(nodeU, V->depot);

        for (auto *nodeV = n(V->depot); !nodeV->isDepot(); nodeV = n(nodeV))
        {
            eval(nodeU, nodeV);  // test inserting U after V
            eval(nodeV, nodeU);  // test inserting V after U
        }
    }

    return bestCost;
}

void RelocateStar::apply(Route *U, Route *V)
{
    std::unordered_set<Node *> seen;
    std::sort(moves.begin(), moves.end());

    for (auto [_, from, to] : moves)
    {
        if (seen.contains(to))  // then our calculation is no longer correct as
            continue;           // we have already altered the insert location.

        from->insertAfter(to);
        seen.insert(p(from));
        seen.insert(from);
        seen.insert(to);
    }
}
