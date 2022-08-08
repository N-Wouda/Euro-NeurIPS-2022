#include "operators.h"

bool moveSingleClient(Node *U, Node *V, Penalties const &penalties)
{
    auto const deltaCost = operators::singleMoveCost(U, V, penalties);

    if (deltaCost < 0)
        U->insertAfter(V);

    return deltaCost < 0;
}
