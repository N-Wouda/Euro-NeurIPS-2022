#include "RelocateStar.h"

int RelocateStar::test(Route *U, Route *V)
{
    bestCost = 0;
    insertionPoint = nullptr;
    nodeToInsert = nullptr;

    auto eval = [&](auto *nodeU, auto *nodeV) {
        int const deltaCost = relocate.test(nodeU, nodeV);

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

    return bestCost;
}
