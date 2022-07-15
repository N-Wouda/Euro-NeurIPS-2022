#ifndef RESULT_H
#define RESULT_H

#include "Individual.h"

#include <vector>

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
    [[nodiscard]] Individual const *getBestFound() const
    {
        // TODO is this the best? See population members
        return feasible[0];  // since they are sorted by increasing cost
    }
};

#endif  // RESULT_H
