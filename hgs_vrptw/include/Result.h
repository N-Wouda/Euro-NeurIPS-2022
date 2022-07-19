#ifndef RESULT_H
#define RESULT_H

#include "Individual.h"

#include <vector>

class Result
{
    Individual const &bestFound;

public:
    Result(Individual const &bestFound) : bestFound(bestFound) {}

    /**
     * Returns the best observed solution.
     */
    [[nodiscard]] Individual const &getBestFound() const { return bestFound; }
};

#endif  // RESULT_H
