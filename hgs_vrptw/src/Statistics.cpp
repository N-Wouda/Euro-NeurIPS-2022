#include "Statistics.h"
#include "Population.h"

#include <algorithm>

void Statistics::collectFrom(Population const &population)
{
    auto const numFeas = std::count_if(
        population.population.begin(),
        population.population.end(),
        [](Individual const *indiv) { return indiv->isFeasible(); });

    numFeasible.push_back(numFeas);
    popSize.push_back(population.population.size());

    auto const &best = population.bestSol;

    if (!best.isFeasible())
        return;

    if (bestObjectives_.empty() || best.cost() < bestObjectives_.back().second)
        bestObjectives_.emplace_back(clock::now(), best.cost());
}
