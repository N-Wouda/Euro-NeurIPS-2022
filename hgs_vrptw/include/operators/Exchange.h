#ifndef EXCHANGE_H
#define EXCHANGE_H

#include "LocalSearchOperator.h"
#include "Node.h"

template <size_t N, size_t M> class Exchange : public LocalSearchOperator<Node>
{
    static_assert(N >= M && N > 0, "N < M or M == 0 does not make sense");

    bool isDepotInSegments(Node *U, Node *V) const;

    bool overlaps(Node *U, Node *V) const;

public:
    bool test(Node *U, Node *V) override;

    void apply(Node *U, Node *V) override;
};

#endif  // EXCHANGE_H
