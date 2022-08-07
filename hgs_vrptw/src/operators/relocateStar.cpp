#include "operators.h"

bool relocateStar(Route *routeU, Route *routeV, Penalties const &penalties)
{
    int bestCost = 0;
    Node *insertionPoint = nullptr;
    Node *nodeToInsert = nullptr;

    for (auto *U = n(routeU->depot); !U->isDepot; U = n(U))
        for (auto *V = n(routeV->depot); !V->isDepot; V = n(V))
        {
            int deltaCost = operators::singleMoveCost(U, V, penalties);

            if (deltaCost < bestCost)
            {
                bestCost = deltaCost;
                insertionPoint = V;
                nodeToInsert = U;
            }
        }

    if (!insertionPoint)
        return false;

    nodeToInsert->insertAfter(insertionPoint);

    return true;
}
