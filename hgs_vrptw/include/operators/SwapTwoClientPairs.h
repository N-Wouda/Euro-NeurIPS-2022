#ifndef SWAPTWOCLIENTPAIRS_H
#define SWAPTWOCLIENTPAIRS_H

#include "LocalSearchOperator.h"
#include "Node.h"

/**
 * Swaps (U -> X) and (V -> Y), if that is an improving move.
 */
class SwapTwoClientPairs : public LocalSearchOperator<Node>
{
public:
    bool test(Node *U, Node *V) override;

    void apply(Node *U, Node *V) override
    {
        n(U)->swapWith(n(V));
        U->swapWith(V);
    }
};

#endif  // SWAPTWOCLIENTPAIRS_H
