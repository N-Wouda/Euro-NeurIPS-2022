#include "RelocateStar.h"

int RelocateStar::evaluate(Route *U, Route *V)
{
    move = {};

    auto eval = [&](Node *N1, Node *N2)
    {
        int const deltaCost = relocate.evaluate(N1, N2);

        if (deltaCost < move.deltaCost)
        {
            move.from = N1;
            move.to = N2;
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

    return move.deltaCost;
}
