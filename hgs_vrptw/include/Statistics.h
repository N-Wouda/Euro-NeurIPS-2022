#ifndef STATISTICS_H
#define STATISTICS_H

#include "Population.h"

#include <chrono>
#include <vector>

class Population;  // forward declaration

class Statistics
{
    using clock = std::chrono::system_clock;
    using timedDatapoints = std::vector<std::pair<clock::time_point, double>>;

    size_t numIters_ = 0;

    // TODO measure and store population diversity statistic?

    std::vector<size_t> popSize;
    std::vector<size_t> numFeasible;
    timedDatapoints bestObjectives_;

public:
    /**
     * Collects population and objective value statistics. This function is
     * called repeatedly during the genetic algorithm's search, and stores
     * relevant data for later evaluation.
     *
     * @param population  Population object to collect data from.
     */
    void collectFrom(Population const &population);

    /**
     * Returns the total number of iterations.
     */
    [[nodiscard]] size_t numIters() const { return numIters_; }

    /**
     * Returns a vector of population sizes, one element for each iteration.
     */
    [[nodiscard]] std::vector<size_t> const &popSizes() const
    {
        return popSize;
    }

    /**
     * Returns a vector of the number of feasible individuals in the population,
     * one element for each iteration. Of course, in each iteration, the number
     * of feasible individuals does not exceed the total population size.
     */
    [[nodiscard]] std::vector<size_t> const &feasiblePops() const
    {
        return numFeasible;
    }

    /**
     * Returns a vector of (datetime, objective)-pairs, one for each time a
     * new, feasible best heuristic solution has been found.
     */
    [[nodiscard]] timedDatapoints const &bestObjectives() const
    {
        return bestObjectives_;
    }
};

#endif  // STATISTICS_H
