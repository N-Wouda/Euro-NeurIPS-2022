#ifndef MAXITERATIONS_H
#define MAXITERATIONS_H

#include "StoppingCriterion.h"

class MaxIterations : public StoppingCriterion
{
    size_t const maxIters;
    size_t currIters = 0;

public:
    bool operator()() override { return maxIters < ++currIters; }

    explicit MaxIterations(size_t const maxIterations) : maxIters(maxIterations)
    {
    }
};

#endif  // MAXITERATIONS_H
