#include "Statistics.h"
#include "Params.h"
#include "Population.h"

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
        [&](double val, Individual const *indiv) {
            return val + indiv->avgBrokenPairsDistance(params.config.nbClose);
        });

    popDiversity_.push_back(totalDiversity / static_cast<double>(nPops));

    auto const &best = population.bestSol;

    if (!best.isFeasible())  // avoids storing the possibly infeasible initial
        return;              // (random) solution

    currObjectives_.push_back(best.cost());

    if (bestObjectives_.empty() || best.cost() < bestObjectives_.back().second)
        bestObjectives_.emplace_back(clock::now(), best.cost());
}

Statistics::Statistics(Params &params) : params(params) {}
