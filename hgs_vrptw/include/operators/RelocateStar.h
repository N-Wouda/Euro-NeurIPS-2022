#ifndef RELOCATESTAR_H
#define RELOCATESTAR_H

#include "Exchange.h"
#include "LocalSearchOperator.h"
#include "Node.h"
#include "Route.h"

/**
 * Performs the best (1, 0)-exchange move between routes U and V. Tests both
 * ways: from U to V, and from V to U.
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

    int evaluate(Route *U, Route *V) override;

    void apply(Route *U, Route *V) override
    {
        if (insertionPoint && nodeToInsert)
            nodeToInsert->insertAfter(insertionPoint);
    }

    explicit RelocateStar(Params const &params)
        : LocalSearchOperator<Route>(params), relocate(params)
    {
    }
};

#endif  // RELOCATESTAR_H
