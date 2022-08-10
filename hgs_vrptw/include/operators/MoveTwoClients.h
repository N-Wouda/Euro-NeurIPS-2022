#ifndef MOVETWOCLIENTS_H
#define MOVETWOCLIENTS_H

#include "LocalSearchOperator.h"
#include "Node.h"

/**
 * Inserts (U -> X) after V, if that is an improving move.
 */
class MoveTwoClients : public LocalSearchOperator<Node>
{
public:
    bool test(Node *U, Node *V) override;

    void apply(Node *U, Node *V) override
    {
        n(U)->insertAfter(V);
        U->insertAfter(V);
    }
};

#endif  // MOVETWOCLIENTS_H
