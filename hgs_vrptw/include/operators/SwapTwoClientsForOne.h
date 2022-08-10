#ifndef SWAPTWOCLIENTSFORONE_H
#define SWAPTWOCLIENTSFORONE_H

#include "LocalSearchOperator.h"
#include "Node.h"

/**
 * Swaps (U -> X) and V, if that is an improving move.
 */
class SwapTwoClientsForOne : public LocalSearchOperator<Node>
{
public:
    bool test(Node *U, Node *V) override;

    void apply(Node *U, Node *V) override
    {
        n(U)->insertAfter(V);
        U->swapWith(V);
    }
};

#endif  // SWAPTWOCLIENTSFORONE_H
