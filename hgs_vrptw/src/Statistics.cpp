#include "Statistics.h"
#include "Population.h"

#include <fstream>
#include <numeric>

void Statistics::collectFrom(Population const &pop)
{
    numIters_++;

    auto const now = clock::now();

    std::chrono::duration<double> const runTime = now - start;
    runTimes_.push_back(runTime.count());

    std::chrono::duration<double> const iterTime = now - lastIter;
    iterTimes_.push_back(iterTime.count());

    lastIter = clock::now();  // update for next call

    auto const opDiversity = [](double val, auto const &subs) {
        return val + subs.indiv->avgBrokenPairsDistanceClosest();
    };

    auto const opCost
        = [](size_t sum, auto const &subs) { return sum + subs.indiv->cost(); };

    auto const opNbRoutes = [&](double val, auto const &subs) {
        return val
               + std::accumulate(subs.indiv->getRoutes().begin(),
                                 subs.indiv->getRoutes().end(),
                                 0,
                                 [](size_t val, auto const &route) {
                                     return val + !route.empty();
                                 });
    };

    if (!pop.feasible.empty())
    {
        auto const numFeas = pop.feasible.size();
        feasStats.popSize_.push_back(numFeas);

        double const feasDiv = std::accumulate(
            pop.feasible.begin(), pop.feasible.end(), 0., opDiversity);
        feasStats.diversity_.push_back(feasDiv / static_cast<double>(numFeas));

        feasStats.bestCost_.push_back(pop.feasible[0].indiv->cost());

        auto const feasCost = std::accumulate(
            pop.feasible.begin(), pop.feasible.end(), 0, opCost);
        feasStats.averageCost_.push_back(feasCost / numFeas);

        auto const feasNbRoutes = std::accumulate(
            pop.feasible.begin(), pop.feasible.end(), 0., opNbRoutes);
        feasStats.nbRoutes_.push_back(feasNbRoutes
                                      / static_cast<double>(numFeas));
    }
    else
    {
        feasStats.popSize_.push_back(0);
        feasStats.diversity_.push_back(0.);  // 0 as substitute for no diversity
        feasStats.bestCost_.push_back(INT_MAX);  // INT_MAX as subst. for inf
        feasStats.averageCost_.push_back(INT_MAX);
        feasStats.nbRoutes_.push_back(0.);
    }

    if (!pop.infeasible.empty())
    {
        auto const numInfeas = pop.infeasible.size();
        infeasStats.popSize_.push_back(numInfeas);

        double const infeasDiv = std::accumulate(
            pop.infeasible.begin(), pop.infeasible.end(), 0., opDiversity);
        infeasStats.diversity_.push_back(infeasDiv
                                         / static_cast<double>(numInfeas));

        infeasStats.bestCost_.push_back(pop.infeasible[0].indiv->cost());

        auto const infeasCost = std::accumulate(
            pop.infeasible.begin(), pop.infeasible.end(), 0, opCost);
        infeasStats.averageCost_.push_back(infeasCost / numInfeas);

        auto const infeasNbRoutes = std::accumulate(
            pop.infeasible.begin(), pop.infeasible.end(), 0., opNbRoutes);
        infeasStats.nbRoutes_.push_back(infeasNbRoutes
                                        / static_cast<double>(numInfeas));
    }
    else
    {
        infeasStats.popSize_.push_back(0);
        infeasStats.diversity_.push_back(0);  // 0 as susbst. for no diversity
        infeasStats.bestCost_.push_back(INT_MAX);  // INT_MAX as subst. for inf
        infeasStats.averageCost_.push_back(INT_MAX);
        infeasStats.nbRoutes_.push_back(0.);
    }

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
