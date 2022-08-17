#include "RelocateStar.h"

int RelocateStar::test(Route *U, Route *V)
{
    bestCost = 0;
    insertionPoint = nullptr;
    nodeToInsert = nullptr;

    auto eval = [&](auto *N1, auto *N2)
    {
        int const deltaCost = relocate.test(N1, N2);

        if (deltaCost < bestCost)
        {
            bestCost = deltaCost;
            insertionPoint = N2;
            nodeToInsert = N1;
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
