#include "Statistics.h"
#include "Params.h"
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
    penaltiesCapacity_.push_back(params.penaltyCapacity);
    penaltiesTimeWarp_.push_back(params.penaltyTimeWarp);

    // Objectives statistics
    auto const &best = population.bestSol;

    if (!best.isFeasible())
        bestObjectives_.push_back(INT_MAX);  // INT_MAX as substitute for inf
    else
        bestObjectives_.push_back(best.cost());

    auto const costFeas
        = accumulate(population.population.begin(),
                     population.population.end(),
                     0,
                     [&](size_t sum, Individual const *indiv) {
                         return indiv->isFeasible() ? sum + indiv->cost() : sum;
                     });

    if (numFeas == 0)
        feasObjectives_.push_back(INT_MAX);
    else
        feasObjectives_.push_back(static_cast<double>(costFeas / numFeas));

    auto const numInfeas = nPops - numFeas;
    auto const costInfeas = accumulate(
        population.population.begin(),
        population.population.end(),
        0,
        [&](size_t sum, Individual const *indiv) {
            return !indiv->isFeasible() ? sum + indiv->cost() : sum;
        });

    if (numInfeas == 0)
        infeasObjectives_.push_back(INT_MAX);
    else
        infeasObjectives_.push_back(
            static_cast<double>(costInfeas / numInfeas));

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
        << "best objective" << sep
        << "feasible avg. objective" << sep
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
            << bestObjectives_[it] << sep
            << feasObjectives_[it] << sep
            << infeasObjectives_[it] << '\n';
    }
    // clang-format on
}
