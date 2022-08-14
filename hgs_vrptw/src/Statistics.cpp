#include "Statistics.h"
#include "Population.h"

#include <fstream>
#include <iostream>
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

    bool foundFeasibleBest = false;
    size_t totalFeasible = 0;
    size_t totalInfeasible = 0;
    size_t nbFeasible = 0;
    size_t nbInfeasible = 0;

    for (auto const &indiv : population.population)
    {
        if (indiv->isFeasible())
        {
            nbFeasible += 1;
            totalFeasible += indiv->cost();

            if (!foundFeasibleBest)
            {
                currObjectives_.push_back(indiv->cost());
                foundFeasibleBest = true;
            }
        }
        else
        {
            nbInfeasible += 1;
            totalInfeasible += indiv->cost();
        }
    }

    if (!foundFeasibleBest)
        currObjectives_.push_back(INT_MAX);  // INT_MAX as substitute for inf

    if (nbFeasible != 0)
        feasibleAvgObjectives_.push_back(
            static_cast<double>(totalFeasible / nbFeasible));
    else
        feasibleAvgObjectives_.push_back(INT_MAX);

    if (nbInfeasible != 0)
        infeasibleAvgObjectives_.push_back(
            static_cast<double>(totalInfeasible / nbInfeasible));
    else
        infeasibleAvgObjectives_.push_back(INT_MAX);

    // Best objectives
    auto const &best = population.bestSol;

    if (!best.isFeasible())
        return;

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
        << sep << "# feasible" << sep << "diversity" << sep
        << "current best objective" << sep << "feasible avg objective" << sep
        << "infeasible avg objective"
        << "\n";

    for (size_t it = 0; it != numIters_; it++)
    {
        out << it << sep << runTimes()[it] << sep << popSizes()[it] << sep
            << feasiblePops()[it] << sep << popDiversity()[it] << sep
            << currObjectives_[it] << sep << feasibleAvgObjectives_[it] << sep
            << infeasibleAvgObjectives_[it] << "\n";
    }
}
