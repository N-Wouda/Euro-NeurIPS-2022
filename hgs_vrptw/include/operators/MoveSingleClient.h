#ifndef MOVESINGLECLIENT_H
#define MOVESINGLECLIENT_H

#include "LocalSearchOperator.h"
#include "Node.h"

/**
 * Inserts U after V, if that is an improving move.
 */
class MoveSingleClient : public LocalSearchOperator<Node>
{
public:
    bool test(Node *U, Node *V) override;

    void apply(Node *U, Node *V) override { U->insertAfter(V); }
};

#endif  // MOVESINGLECLIENT_H
