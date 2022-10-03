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

    // Population statistics
    auto const numFeas = pop.feasible.size();
    auto const numInfeas = pop.infeasible.size();
    feasPopSize_.push_back(numFeas);
    infeasPopSize_.push_back(numInfeas);

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
                                 };);
    };

    if (!pop.feasible.empty())
    {
        double const feasDiv = std::accumulate(
            pop.feasible.begin(), pop.feasible.end(), 0., opDiversity);
        feasDiversity_.push_back(feasDiv / static_cast<double>(numFeas));

        feasBest_.push_back(pop.feasible[0].indiv->cost());

        auto const feasCost
            = accumulate(pop.feasible.begin(), pop.feasible.end(), 0, opCost);
        feasAverage_.push_back(feasCost / numFeas);

        auto const feasNbRoutes = accumulate(
            pop.feasible.begin(), pop.feasible.end(), 0., opNbRoutes);
        feasNbRoutes_.push_back(feasNbRoutes / static_cast<double>(numFeas));
    }
    else
    {
        feasDiversity_.push_back(0.);  // 0 as substitute for no diversity
        feasBest_.push_back(INT_MAX);  // INT_MAX as substitute for inf
        feasAverage_.push_back(INT_MAX);
        feasNbRoutes_.push_back(0.);
    }

    if (!pop.infeasible.empty())
    {
        double const infeasDiv = std::accumulate(
            pop.infeasible.begin(), pop.infeasible.end(), 0., opDiversity);
        infeasDiversity_.push_back(infeasDiv / static_cast<double>(numInfeas));

        infeasBest_.push_back(pop.infeasible[0].indiv->cost());

        auto const infeasCost = accumulate(
            pop.infeasible.begin(), pop.infeasible.end(), 0, opCost);
        infeasAverage_.push_back(infeasCost / numInfeas);

        auto const infeasNbRoutes = accumulate(
            pop.infeasible.begin(), pop.infeasible.end(), 0., opNbRoutes);
        infeasNbRoutes_.push_back(infeasNbRoutes
                                  / static_cast<double>(numInfeas));
    }
    else
    {
        infeasDiversity_.push_back(0);   // 0 as substitute for no diversity
        infeasBest_.push_back(INT_MAX);  // INT_MAX as substitute for inf
        infeasAverage_.push_back(INT_MAX);
        infeasNbRoutes_.push_back(0.);
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
            << feasPopSize_[it] << sep
            << feasDiversity_[it] << sep
            << feasBest_[it] << sep
            << feasAverage_[it] << sep
            << feasNbRoutes_[it] << sep
            << infeasPopSize_[it] << sep
            << infeasDiversity_[it] << sep
            << infeasBest_[it] << sep
            << infeasAverage_[it] << sep
            << infeasNbRoutes_[it] << sep
            << penaltiesCapacity_[it] << sep
            << penaltiesTimeWarp_[it] << '\n';
    }
    // clang-format on
}
