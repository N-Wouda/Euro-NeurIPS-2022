#include "Statistics.h"
#include "Population.h"

#include <fstream>
#include <numeric>

namespace
{
void collectSubPopStats(Population::SubPopulation const &subPop,
                        Statistics::SubPopStats &subStats)
{
    if (!subPop.empty())
    {
        auto const popSize = subPop.size();
        subStats.popSize_.push_back(popSize);

        auto const opDiversity = [](double val, auto const &subs) {
            return val + subs.indiv->avgBrokenPairsDistanceClosest();
        };
        subStats.diversity_.push_back(
            std::accumulate(subPop.begin(), subPop.end(), 0., opDiversity)
            / popSize);

        subStats.bestCost_.push_back(subPop[0].indiv->cost());

        auto const opAverageCost = [](size_t sum, auto const &subs) {
            return sum + subs.indiv->cost();
        };
        subStats.averageCost_.push_back(
            std::accumulate(subPop.begin(), subPop.end(), 0, opAverageCost)
            / popSize);

        auto const opCountNonEmptyRoutes = [](size_t val, auto const &route) {
            return val + !route.empty();
        };
        auto const opNbRoutes = [&](double val, auto const &subs) {
            return val
                   + std::accumulate(subs.indiv->getRoutes().begin(),
                                     subs.indiv->getRoutes().end(),
                                     0,
                                     opCountNonEmptyRoutes);
        };
        subStats.nbRoutes_.push_back(
            std::accumulate(subPop.begin(), subPop.end(), 0., opNbRoutes)
            / popSize);
    }
    else
    {
        subStats.popSize_.push_back(0);
        subStats.diversity_.push_back(0.);  // 0 as substitute for no diversity
        subStats.bestCost_.push_back(INT_MAX);  // INT_MAX as subst. for inf
        subStats.averageCost_.push_back(INT_MAX);
        subStats.nbRoutes_.push_back(0.);
    }
}
}  // namespace

void Statistics::collectFrom(Population const &pop)
{
    numIters_++;

    auto const now = clock::now();

    std::chrono::duration<double> const runTime = now - start;
    runTimes_.push_back(runTime.count());

    std::chrono::duration<double> const iterTime = now - lastIter;
    iterTimes_.push_back(iterTime.count());

    lastIter = clock::now();  // update for next call

    // Population statistics
    collectSubPopStats(pop.feasible, feasStats);
    collectSubPopStats(pop.infeasible, infeasStats);

    // Penalty statistics
    penaltiesCapacity_.push_back(pop.params.penaltyCapacity);
    penaltiesTimeWarp_.push_back(pop.params.penaltyTimeWarp);

    // Incumbents
    auto const &best = pop.bestSol;

    if (!best.isFeasible())
        return;

    if (incumbents_.empty() || best.cost() < incumbents_.back().second)
    {
        std::chrono::duration<double> time = clock::now() - start;
        incumbents_.emplace_back(time.count(), best.cost());
    }
}

void Statistics::toCsv(std::string const &path, char const sep) const
{
    std::ofstream out(path);

    if (!out)
        throw std::runtime_error("Could not open " + path);

    // clang-format off
    out << "total run-time (s)" << sep
        << "iteration run-time (s)" << sep
        << "# feasible" << sep
        << "feasible diversity" << sep
        << "feasible best objective" << sep
        << "feasible avg. objective" << sep
        << "feasible avg. # routes" << sep
        << "# infeasible" << sep
        << "infeasible diversity" << sep
        << "infeasible best. objective" << sep
        << "infeasible avg. objective" << sep
        << "infeasible avg. # routes" << sep
        << "penalty capacity" << sep
        << "penalty time warp" << '\n';

    for (size_t it = 0; it != numIters_; it++)
    {
        out << runTimes_[it] << sep
            << iterTimes_[it] << sep
            << feasStats.popSize_[it] << sep
            << feasStats.diversity_[it] << sep
            << feasStats.bestCost_[it] << sep
            << feasStats.averageCost_[it] << sep
            << feasStats.nbRoutes_[it] << sep
            << infeasStats.popSize_[it] << sep
            << infeasStats.diversity_[it] << sep
            << infeasStats.bestCost_[it] << sep
            << infeasStats.averageCost_[it] << sep
            << infeasStats.nbRoutes_[it] << sep
            << penaltiesCapacity_[it] << sep
            << penaltiesTimeWarp_[it] << '\n';
    }
    // clang-format on
}
