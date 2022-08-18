#ifndef STATISTICS_H
#define STATISTICS_H

#include "Population.h"

#include <chrono>
#include <vector>

class Population;  // forward declaration

class Statistics
{
    using clock = std::chrono::system_clock;
    using timedDatapoints = std::vector<std::pair<double, size_t>>;

    Params &params;

    clock::time_point start = clock::now();
    clock::time_point lastIter = clock::now();
    size_t numIters_ = 0;

    std::vector<size_t> currIters_;
    std::vector<double> runTimes_;
    std::vector<double> iterTimes_;
    std::vector<size_t> popSizes_;
    std::vector<size_t> numFeasiblePop_;
    std::vector<double> popDiversity_;
    std::vector<size_t> bestObjectives_;
    std::vector<double> feasObjectives_;
    std::vector<double> infeasObjectives_;

    timedDatapoints incumbents_;

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
     * Returns the total number of collected iterations.
     */
    [[nodiscard]] size_t numIters() const { return numIters_; }

    /**
     * Returns a vector of integers, indicating at which iteration of the
     * algorithm the statistics were collected.
     */
    [[nodiscard]] std::vector<size_t> const &currIters() const
    {
        return currIters_;
    }

    /**
     * Returns a vector of run times in seconcds, indicating the elapsed time
     * since the start of the algorithm.
     */
    [[nodiscard]] std::vector<double> const &runTimes() const
    {
        return runTimes_;
    }

    /**
     * Returns a vector of run times in seconds, indicating the elapsed time
     * between the current and last collected iteration.
     */
    [[nodiscard]] std::vector<double> const &iterTimes() const
    {
        return iterTimes_;
    }

    /**
     * Returns a vector of population sizes, one element for each collected
     * iteration.
     */
    [[nodiscard]] std::vector<size_t> const &popSizes() const
    {
        return popSizes_;
    }

    /**
     * Returns a vector of the number of feasible individuals in the population,
     * one element for each collected iteration. Of course, in each iteration,
     * the number of feasible individuals does not exceed the total population
     * size.
     */
    [[nodiscard]] std::vector<size_t> const &numFeasiblePop() const
    {
        return numFeasiblePop_;
    }

    /**
     * Returns a vector of the average population diversity, one element for
     * each iteration. The average diversity is computed as the average broken
     * pairs distance for each individual in the population, compared to its
     * neighbours (the neighbourhood size is controlled by the ``nbClose``
     * setting).
     */
    [[nodiscard]] std::vector<double> const &popDiversity() const
    {
        return popDiversity_;
    }

    /**
     * Returns a vector of the best objective value at the current iteration,
     * one element for each iteration where a feasible best solution exists.
     * Early iterations where that might not be the case are stored with
     * value ``INT_MAX`` as substitute for inf.
     */
    [[nodiscard]] std::vector<size_t> const &bestObjectives() const
    {
        return bestObjectives_;
    }

    /**
     * Returns a vector of the average objective value of the feasible
     * individuals at the current iteration.
     * */

    [[nodiscard]] std::vector<double> const &feasObjectives() const
    {
        return feasObjectives_;
    }

    /**
     * Returns a vector of the average objective value of the infeasible
     * individuals at the current iteration.
     * */

    [[nodiscard]] std::vector<double> const &infeasObjectives() const
    {
        return infeasObjectives_;
    }

    /**
     * Returns a vector of (datetime, objective)-pairs, one for each time a
     * new, feasible best heuristic solution has been found.
     */
    [[nodiscard]] timedDatapoints const &incumbents() const
    {
        return incumbents_;
    }

    /**
     * Exports the collected statistics as CSV. Only statistics that were
     * collected for each iteration are exported. Uses `,` as default
     * separator.
     */
    void toCsv(std::string const &path, char const sep = ',') const;

    Statistics(Params &params);
};

#endif  // STATISTICS_H
