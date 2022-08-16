#ifndef RELOCATESTAR_H
#define RELOCATESTAR_H

#include "Exchange.h"
#include "LocalSearchOperator.h"
#include "Node.h"
#include "Route.h"

/**
 * Performs the best (1, 0)-exchange move between routes U and V.
 */
class RelocateStar : public LocalSearchOperator<Route>
{
    Exchange<1, 0> relocate;

    int bestCost = 0;
    Node *insertionPoint = nullptr;
    Node *nodeToInsert = nullptr;

public:
    void init(Individual const &indiv, Penalties const *penalties) override
    {
        LocalSearchOperator<Route>::init(indiv, penalties);
        relocate.init(indiv, penalties);
    }

    int test(Route *U, Route *V) override;

    void apply(Route *U, Route *V) override
    {
        if (insertionPoint && nodeToInsert)
            nodeToInsert->insertAfter(insertionPoint);
    }
};

#endif  // RELOCATESTAR_H
