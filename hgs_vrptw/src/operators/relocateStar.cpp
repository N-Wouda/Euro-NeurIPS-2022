#include "operators.h"

bool relocateStar(Route *routeU, Route *routeV, Penalties const &penalties)
{
    int bestCost = 0;
    Node *insertionPoint = nullptr;
    Node *nodeToInsert = nullptr;

    auto eval = [&](auto *U, auto *V)
    {
        int const deltaCost = operators::singleMoveCost(U, V, penalties);

        if (deltaCost < bestCost)
        {
            bestCost = deltaCost;
            insertionPoint = V;
            nodeToInsert = U;
        }
    };

    for (auto *U = n(routeU->depot); !U->isDepot; U = n(U))
    {
        eval(U, routeV->depot);

        for (auto *V = n(routeV->depot); !V->isDepot; V = n(V))
            eval(U, V);
    }

    if (!insertionPoint)
        return false;

    nodeToInsert->insertAfter(insertionPoint);

    return true;
}
