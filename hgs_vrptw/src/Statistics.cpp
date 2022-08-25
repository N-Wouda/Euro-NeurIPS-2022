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

    // Penalty statistics
    penaltiesCapacity_.push_back(pop.params.penaltyCapacity);
    penaltiesTimeWarp_.push_back(pop.params.penaltyTimeWarp);

    // Population statistics
    auto const numFeas = pop.feasible.individuals.size();
    auto const numInfeas = pop.infeasible.individuals.size();
    popSizes_.push_back(numFeas + numInfeas);
    numFeasiblePop_.push_back(numFeas);

    auto const opDiversity = [](double val, Individual const *indiv) {
        return val + indiv->avgBrokenPairsDistanceClosest();
    };

    double const feasDiv = std::accumulate(pop.feasible.individuals.begin(),
                                           pop.feasible.individuals.end(),
                                           0,
                                           opDiversity);
    feasDiversity_.push_back(feasDiv / static_cast<double>(numFeas));

    double const infeasDiv = std::accumulate(pop.infeasible.individuals.begin(),
                                             pop.infeasible.individuals.end(),
                                             0,
                                             opDiversity);
    infeasDiversity_.push_back(infeasDiv / static_cast<double>(numInfeas));

    // Objectives statistics
    auto const opCost = [](size_t sum, Individual const *indiv) {
        return sum + indiv->cost();
    };

    if (pop.feasible.individuals.size() > 0)
    {
        feasBest_.push_back(pop.feasible.individuals[0]->cost());
        auto const costFeas = accumulate(pop.feasible.individuals.begin(),
                                         pop.feasible.individuals.end(),
                                         0,
                                         opCost);
        feasAverage_.push_back(costFeas / numFeas);
    }
    else
    {
        feasBest_.push_back(INT_MAX);  // INT_MAX as substitute for inf
        feasAverage_.push_back(INT_MAX);
    }

    if (pop.infeasible.individuals.size() > 0)
    {
        feasBest_.push_back(pop.infeasible.individuals[0]->cost());
        auto const costFeas = accumulate(pop.infeasible.individuals.begin(),
                                         pop.infeasible.individuals.end(),
                                         0,
                                         opCost);
        feasAverage_.push_back(costFeas / numFeas);
    }
    else
    {
        infeasBest_.push_back(INT_MAX);  // INT_MAX as substitute for inf
        infeasAverage_.push_back(INT_MAX);
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
        << "population size" << sep
        << "# feasible" << sep
        << "infeasible diversity" << sep
        << "feasible diversity" << sep
        << "feasible best objective" << sep
        << "feasible avg. objective" << sep
        << "infeasible best. objective" << sep
        << "infeasible avg. objective" << sep
        << "penalty capacity" << sep
        << "penalty time warp" << '\n';

    for (size_t it = 0; it != numIters_; it++)
    {
        out << runTimes_[it] << sep
            << iterTimes_[it] << sep
            << popSizes_[it] << sep
            << numFeasiblePop_[it] << sep
            << feasDiversity_[it] << sep
            << infeasDiversity_[it] << sep
            << feasBest_[it] << sep
            << feasAverage_[it] << sep
            << infeasBest_[it] << sep
            << infeasAverage_[it] << sep
            << penaltiesCapacity_[it] << sep
            << penaltiesTimeWarp_[it] << '\n';
    }
    // clang-format on
}
