#ifndef REVERSEEXCHANGE_H
#define REVERSEEXCHANGE_H

#include "LocalSearchOperator.h"
#include "Node.h"

/**
 * Template class that exchanges N consecutive nodes from U's route (starting at
 * U) with M consecutive nodes from V's route (starting at V), while reversing
 * the direction of the nodes.
 */
template <size_t N, size_t M>
class ReverseExchange : public LocalSearchOperator<Node>
{
    static_assert(N >= M && N > 0, "N < M or M == 0 does not make sense");

    // Returns the last nodes to be moved in the segments of U and V
    std::pair<Node *, Node *> getEnds(Node *U, Node *V) const;

    // Tests if the to-be-moved segments of U or V contain the depot
    inline bool isDepotInSegments(Node *U, Node *V) const;

    // Tests if the segments of U and V overlap in the same route
    inline bool overlaps(Node *U, Node *V) const;

    // Tests if the segments of U and V are adjacent in the same route
    inline bool adjacent(Node *U, Node *V) const;

    // Special case that's applied when M == 0
    int testPureMove(Node *U, Node *V) const;

    // Applied when M != 0
    int testSwapMove(Node *U, Node *V) const;

public:
    int test(Node *U, Node *V) override;

    void apply(Node *U, Node *V) override;
};

#endif  // REVERSEEXCHANGE_H
