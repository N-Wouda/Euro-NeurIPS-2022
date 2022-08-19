#ifndef RELOCATESTAR_H
#define RELOCATESTAR_H

#include "Exchange.h"
#include "LocalSearchOperator.h"
#include "Node.h"
#include "Route.h"

#include <unordered_set>
#include <vector>

/**
 * Performs the best (1, 0)-exchange move between routes U and V. Tests both
 * ways: from U to V, and from V to U.
 */
class RelocateStar : public LocalSearchOperator<Route>
{
    struct Move
    {
        int deltaCost = 0;
        Node *from = nullptr;
        Node *to = nullptr;

        bool operator<(Move const &other) const
        {
            return deltaCost < other.deltaCost;
        }

        Move(int deltaCost, Node *from, Node *to)
            : deltaCost(deltaCost), from(from), to(to)
        {
        }
    };

    Exchange<1, 0> relocate;
    std::vector<Move> moves;

public:
    void init(Individual const &indiv, Penalties const *penalties) override
    {
        LocalSearchOperator<Route>::init(indiv, penalties);
        relocate.init(indiv, penalties);
    }

    int evaluate(Route *U, Route *V) override;

    void apply(Route *U, Route *V) override;

    explicit RelocateStar(Params const &params)
        : LocalSearchOperator<Route>(params), relocate(params)
    {
    }
};

#endif  // RELOCATESTAR_H
