#include "operators.h"

bool relocateStar(Route *routeU, Route *routeV, Penalties const &penalties)
{
    int bestCost = 0;
    Node *insertionPoint = nullptr;
    Node *nodeToInsert = nullptr;

    for (auto *U = routeU->depot->next; !U->isDepot; U = U->next)
        for (auto *V = routeV->depot->next; !V->isDepot; V = V->next)
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
