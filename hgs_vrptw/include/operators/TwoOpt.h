#ifndef TWOOPT_H
#define TWOOPT_H

#include "LocalSearchOperator.h"
#include "Node.h"

class TwoOpt : public LocalSearchOperator<Node>
{
    bool withinRouteTest(Node *U, Node *V);

    bool betweenRouteTest(Node *U, Node *V);

    void withinRouteApply(Node *U, Node *V);

    void betweenRouteApply(Node *U, Node *V);

public:
    bool test(Node *U, Node *V) override;

    void apply(Node *U, Node *V) override;
};

#endif  // TWOOPT_H
