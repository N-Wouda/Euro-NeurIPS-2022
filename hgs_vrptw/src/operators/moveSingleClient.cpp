#include "operators.h"

bool moveSingleClient(Node *nodeU, Node *nodeV, Penalties const &penalties)
{
    auto const deltaCost = operators::singleMoveCost(nodeU, nodeV, penalties);

    if (deltaCost < 0)
        nodeU->insertAfter(nodeV);

    return deltaCost < 0;
}
