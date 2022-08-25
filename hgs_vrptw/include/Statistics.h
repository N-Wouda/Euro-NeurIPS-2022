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

    clock::time_point start = clock::now();
    clock::time_point lastIter = clock::now();
    size_t numIters_ = 0;

    std::vector<double> runTimes_;
    std::vector<double> iterTimes_;
    std::vector<size_t> popSizes_;
    std::vector<size_t> numFeasiblePop_;
    std::vector<double> feasDiversity_;
    std::vector<double> infeasDiversity_;
    std::vector<size_t> feasBest_;
    std::vector<size_t> feasAverage_;
    std::vector<size_t> infeasBest_;
    std::vector<size_t> infeasAverage_;
    std::vector<size_t> penaltiesCapacity_;
    std::vector<size_t> penaltiesTimeWarp_;

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
     * Returns the total number of iterations.
     */
    [[nodiscard]] size_t numIters() const { return numIters_; }

    /**
     * Returns a vector of run times in seconds, one element per iteration.
     * Each element indicates the time between the current iteration and the
     * start of the algorithm.
     */
    [[nodiscard]] std::vector<double> const &runTimes() const
    {
        return runTimes_;
    }

    /**
     * Returns a vector of run times in seconds, one element per iteration.
     * Each element indicates the time between the current and previous
     * iteration.
     */
    [[nodiscard]] std::vector<double> const &iterTimes() const
    {
        return iterTimes_;
    }

    /**
     * Returns a vector of population sizes, one element per iteration.
     */
    [[nodiscard]] std::vector<size_t> const &popSizes() const
    {
        return popSizes_;
    }

    /**
     * Returns a vector of the number of feasible individuals in the population,
     * one element per iteration. Of course, in each iteration, the number of
     * feasible individuals does not exceed the total population size.
     */
    [[nodiscard]] std::vector<size_t> const &numFeasiblePop() const
    {
        return numFeasiblePop_;
    }

    /**
     * Returns a vector of the average feasible subpopulation diversity, one
     * element per iteration. The average diversity is computed as the average
     * broken pairs distance for each individual in the subpopulation, compared
     * to its neighbours (the neighbourhood size is controlled by the
     * ``nbClose`` setting).
     */
    [[nodiscard]] std::vector<double> const &feasDiversity() const
    {
        return feasDiversity_;
    }

    /**
     * Returns a vector of the average infeasible subpopulation diversity, one
     * element per iteration. The average diversity is computed as the average
     * broken pairs distance for each individual in the subpopulation, compared
     * to its neighbours (the neighbourhood size is controlled by the
     * ``nbClose`` setting).
     */
    [[nodiscard]] std::vector<double> const &infeasDiversity() const
    {
        return infeasDiversity_;
    }

    /**
     * Returns a vector of the best objective value of feasible individuals,
     * one element per iteration. If there are no feasible individuals, then
     * ``INT_MAX`` is stored.
     */
    [[nodiscard]] std::vector<size_t> const &feasBest() const
    {
        return feasBest_;
    }

    /**
     * Returns a vector of the average objective value of feasible individuals,
     * one element per iteration. If there are no feasible individuals, then
     * ``INT_MAX`` is stored.
     */
    [[nodiscard]] std::vector<size_t> const &feasAverage() const
    {
        return feasAverage_;
    }

    /**
     * Returns a vector of the best objective value of infeasible individuals,
     * one element per iteration. If there are no infeasible individuals, then
     * ``INT_MAX`` is stored.
     */
    [[nodiscard]] std::vector<size_t> const &infeasBest() const
    {
        return infeasBest_;
    }

    /**
     * Returns a vector of the average objective value of infeasible
     * individuals, one element per iteration. If there are no infeasible
     * individuals, then ``INT_MAX`` is stored.
     */
    [[nodiscard]] std::vector<size_t> const &infeasAverage() const
    {
        return infeasAverage_;
    }

    /**
     * Returns a vector of capacity penalties, one element per iteration.
     */
    [[nodiscard]] std::vector<size_t> const &penaltiesCapacity() const
    {
        return penaltiesCapacity_;
    }

    /**
     * Returns a vector of time warp penalties, one element per iteration.
     */
    [[nodiscard]] std::vector<size_t> const &penaltiesTimeWarp() const
    {
        return penaltiesTimeWarp_;
    }

    /**
     * Returns a vector of (runtime, objective)-pairs, one for each time
     * a new, feasible best heuristic solution has been found.
     */
    [[nodiscard]] timedDatapoints const &incumbents() const
    {
        return incumbents_;
    }

    /**
     * Exports the collected statistics as CSV. Only statistics that have been
     * collected for each iteration are exported. Uses `,` as default separator.
     */
    void toCsv(std::string const &path, char const sep = ',') const;
};

#endif  // STATISTICS_H
