#ifndef RESULT_H
#define RESULT_H

#include "Individual.h"
#include <iosfwd>

class Result
{
    using SubPopulation = std::vector<Individual *>;

    SubPopulation const &feasible;
    SubPopulation const &infeasible;

public:
    Result(SubPopulation const &feasible, SubPopulation const &infeasible)
        : feasible(feasible), infeasible(infeasible)
    {
    }

    /**
     * Returns the best observed solution.
     */
    [[nodiscard]] Individual getBestFound() const
    {
        // TODO why not allowed to pass pointer?
        return *feasible[0];  // since they are sorted by increasing cost
    }
};

#endif  // RESULT_H
