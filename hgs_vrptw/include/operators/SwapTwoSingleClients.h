#ifndef SWAPTWOSINGLECLIENTS_H
#define SWAPTWOSINGLECLIENTS_H

#include "LocalSearchOperator.h"
#include "Node.h"

/**
 * Swaps U and V, if that is an improving move.
 */
class SwapTwoSingleClients : public LocalSearchOperator<Node>
{
public:
    bool test(Node *U, Node *V) override;

    void apply(Node *U, Node *V) override { U->swapWith(V); }
};

#endif  // SWAPTWOSINGLECLIENTS_H
