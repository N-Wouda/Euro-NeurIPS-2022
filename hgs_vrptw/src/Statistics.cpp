#include "Statistics.h"
#include "Population.h"

#include <fstream>
#include <numeric>

void Statistics::collectFrom(Population const &population)
{
    numIters_++;

    std::chrono::duration<double> diff = clock::now() - lastIter;
    iterTimes.push_back(diff.count());
    lastIter = clock::now();  // update for next call

    auto const numFeas = std::count_if(
        population.population.begin(),
        population.population.end(),
        [](Individual const *indiv) { return indiv->isFeasible(); });

    numFeasible.push_back(numFeas);

    auto const nPops = population.population.size();
    popSize.push_back(nPops);

    double const totalDiversity = std::accumulate(
        population.population.begin(),
        population.population.end(),
        0.,
        [](double val, Individual const *indiv) {
            return val + indiv->avgBrokenPairsDistanceClosest();
        });

    popDiversity_.push_back(totalDiversity / static_cast<double>(nPops));

    auto const &best = population.bestSol;

    if (!best.isFeasible())
    {
        currObjectives_.push_back(INT_MAX);  // INT_MAX as substitute for inf
        return;
    }

    currObjectives_.push_back(best.cost());

    if (bestObjectives_.empty() || best.cost() < bestObjectives_.back().second)
        bestObjectives_.emplace_back(clock::now(), best.cost());
}

void Statistics::toCsv(std::string const &path, char const sep) const
{
    std::ofstream out(path);

    if (!out)
        throw std::runtime_error("Could not open " + path);

    out << "# iteration" << sep << "run-time (s)" << sep << "population size"
        << sep << "# feasible" << sep << "diversity" << sep << "best objective"
        << "\n";

    for (size_t it = 0; it != numIters_; it++)
    {
        out << it << sep << runTimes()[it] << sep << popSizes()[it] << sep
            << feasiblePops()[it] << sep << popDiversity()[it] << sep
            << currObjectives_[it] << "\n";
    }
}
