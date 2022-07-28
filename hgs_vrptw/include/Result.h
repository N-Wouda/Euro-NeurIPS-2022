#ifndef RESULT_H
#define RESULT_H

#include "Individual.h"
#include "Statistics.h"

#include <utility>
#include <vector>

class Result
{
    Individual const &bestFound;
    size_t numIters;
    Statistics stats;

public:
    Result(Individual const &bestFound, size_t numIters, Statistics stats)
        : bestFound(bestFound), numIters(numIters), stats(std::move(stats))
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

    /**
     * Returns statistics collected by the genetic algorithm.
     */
    [[nodiscard]] Statistics const &getStatistics() const { return stats; }
};

#endif  // RESULT_H
