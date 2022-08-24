#include "Statistics.h"
#include "Population.h"

#include <fstream>
#include <numeric>

void Statistics::collectFrom(Population const &population)
{
    numIters_++;

    auto const now = clock::now();

    std::chrono::duration<double> const runTime = now - start;
    runTimes_.push_back(runTime.count());

    std::chrono::duration<double> const iterTime = now - lastIter;
    iterTimes_.push_back(iterTime.count());

    lastIter = clock::now();  // update for next call

    // Population statistics
    auto const nPops = population.population.size();
    popSizes_.push_back(nPops);

    auto const numFeas = std::count_if(
        population.population.begin(),
        population.population.end(),
        [](Individual const *indiv) { return indiv->isFeasible(); });

    numFeasiblePop_.push_back(numFeas);

    double const totalDiversity = std::accumulate(
        population.population.begin(),
        population.population.end(),
        0.,
        [](double val, Individual const *indiv) {
            return val + indiv->avgBrokenPairsDistanceClosest();
        });

    popDiversity_.push_back(totalDiversity / static_cast<double>(nPops));

    // Penalty statistics
    penaltiesCapacity_.push_back(population.params.penaltyCapacity);
    penaltiesTimeWarp_.push_back(population.params.penaltyTimeWarp);

    // Objectives statistics
    auto itrFeas = std::find_if(
        population.population.begin(),
        population.population.end(),
        [](Individual const *indiv) { return indiv->isFeasible(); });

    if (itrFeas != population.population.end())
    {
        feasBest_.push_back((*itrFeas)->cost());

        auto const costFeas = accumulate(
            population.population.begin(),
            population.population.end(),
            0,
            [&](size_t sum, Individual const *indiv) {
                return indiv->isFeasible() ? sum + indiv->cost() : sum;
            });

        feasAverage_.push_back(costFeas / numFeas);
    }
    else
    {
        feasBest_.push_back(INT_MAX);  // INT_MAX as substitute for inf
        feasAverage_.push_back(INT_MAX);
    }

    auto itrInfeas = std::find_if(
        population.population.begin(),
        population.population.end(),
        [](Individual const *indiv) { return !indiv->isFeasible(); });

    if (itrInfeas != population.population.end())
    {
        infeasBest_.push_back((*itrInfeas)->cost());

        auto const numInfeas = nPops - numFeas;
        auto const costInfeas = accumulate(
            population.population.begin(),
            population.population.end(),
            0,
            [&](size_t sum, Individual const *indiv) {
                return !indiv->isFeasible() ? sum + indiv->cost() : sum;
            });

        infeasAverage_.push_back(costInfeas / numInfeas);
    }
    else
    {
        infeasBest_.push_back(INT_MAX);  // INT_MAX as substitute for inf
        infeasAverage_.push_back(INT_MAX);
    }

    auto const &best = population.bestSol;

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
        << "diversity" << sep
        << "penalty capacity" << sep
        << "penalty time warp" << sep
        << "feasible best objective" << sep
        << "feasible avg. objective" << sep
        << "infeasible best. objective" << sep
        << "infeasible avg. objective" << '\n';

    for (size_t it = 0; it != numIters_; it++)
    {
        out << runTimes_[it] << sep
            << iterTimes_[it] << sep
            << popSizes_[it] << sep
            << numFeasiblePop_[it] << sep
            << popDiversity_[it] << sep
            << penaltiesCapacity_[it] << sep
            << penaltiesTimeWarp_[it] << sep
            << feasBest_[it] << sep
            << feasAverage_[it] << sep
            << infeasBest_[it] << sep
            << infeasAverage_[it] << '\n';
    }
    // clang-format on
}
