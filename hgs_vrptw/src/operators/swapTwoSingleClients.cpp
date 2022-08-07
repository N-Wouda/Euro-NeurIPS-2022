#include "operators.h"

bool swapTwoSingleClients(Node *nodeU, Node *nodeV, Penalties const &penalties)
{
    auto const deltaCost = operators::twoSwapCost(nodeU, nodeV, penalties);

    if (deltaCost < 0)
        nodeU->swapWith(nodeV);

    return deltaCost < 0;
}
