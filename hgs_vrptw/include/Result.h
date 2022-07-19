#ifndef RESULT_H
#define RESULT_H

#include "Individual.h"

#include <vector>

class Result
{
    Individual const &bestFound;
    size_t numIters;

public:
    Result(Individual const &bestFound, size_t numIters)
        : bestFound(bestFound), numIters(numIters)
    {
    }

    /**
     * Returns the best observed solution.
     */
    [[nodiscard]] Individual const &getBestFound() const { return bestFound; }

    /**
     * Returns the total number of iterations.
     */
    [[nodiscard]] size_t getNumIters() const { return numIters; }
};

#endif  // RESULT_H
