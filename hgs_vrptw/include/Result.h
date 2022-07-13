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
     * Exports the best known feasible solution to the location indicated by
     * `path`.
     */
    void writeBestKnowSolution(std::string const &path) const;

    [[nodiscard]] Individual const *getBestFound() const;
};

#endif  // RESULT_H
