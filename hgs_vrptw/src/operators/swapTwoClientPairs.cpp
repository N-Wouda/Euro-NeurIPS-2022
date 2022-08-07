#include "operators.h"

bool swapTwoClientPairs(Node *nodeU, Node *nodeV, Penalties const &penalties)
{
    auto const deltaCost = operators::twoSwapCost(nodeU, nodeV, penalties);

    if (deltaCost < 0)
    {
        nodeU->next->insertAfter(nodeV);
        nodeU->swapWith(nodeV);
    }

    return deltaCost < 0;
}
