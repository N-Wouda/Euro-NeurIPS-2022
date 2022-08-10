#ifndef RELOCATESTAR_H
#define RELOCATESTAR_H

#include "LocalSearchOperator.h"
#include "Node.h"
#include "Route.h"

/**
 * Performs the best moveSingleClient move between routes U and V.
 */
class RelocateStar : public LocalSearchOperator<Route>
{
    int bestCost = 0;
    Node *insertionPoint = nullptr;
    Node *nodeToInsert = nullptr;

    int singleMoveCost(Node *U, Node *V) const;

public:
    bool test(Route *U, Route *V) override;

    void apply(Route *U, Route *V) override
    {
        if (insertionPoint && nodeToInsert)
            nodeToInsert->insertAfter(insertionPoint);
    }
};

#endif  // RELOCATESTAR_H
