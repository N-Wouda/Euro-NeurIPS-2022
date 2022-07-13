#ifndef RESULT_H
#define RESULT_H

#include "Individual.h"
#include <iosfwd>

class Result
{
    using SubPopulation = std::vector<Individual const *>;

    SubPopulation const &feasible;
    SubPopulation const &infeasible;

public:
    Result(SubPopulation const &feasible, SubPopulation const &infeasible)
        : feasible(feasible), infeasible(infeasible)
    {
    }

    void exportBestKnownSolution(std::string const &path) const;

    [[nodiscard]] Individual const *getBestFound() const;
};

#endif  // RESULT_H
