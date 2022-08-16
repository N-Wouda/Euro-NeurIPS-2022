#include "GeneticAlgorithm.h"

#include "Individual.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Result.h"
#include "Statistics.h"

#include <numeric>
#include <stdexcept>

Result GeneticAlgorithm::run(StoppingCriterion &stop)
{
    if (operators.empty())
        throw std::runtime_error("Cannot run genetic algorithm without "
                                 "crossover operators.");

    if (params.nbClients == 1)
        throw std::runtime_error("Cannot run genetic algorithm with one node.");

    Statistics stats;

    size_t iter = 0;
    size_t nbIterNonProd = 1;

    while (not stop())
    {
        iter++;

        if (nbIterNonProd == params.config.nbIter)  // restart population after
        {                                           // this number of useless
            population.restart();                   // iterations
            nbIterNonProd = 1;
        }

        auto const currBest = population.getBestFound().cost();

        auto offspring = crossover();
        educate(offspring);

        if (currBest > population.getBestFound().cost())  // has new best!
            nbIterNonProd = 1;
        else
            nbIterNonProd++;

        // Diversification and penalty management
        if (iter % params.config.nbPenaltyManagement == 0)
        {
            updatePenalties();
            population.reorder();  // re-order since penalties have changed
        }

        if (params.config.collectStatistics)
            stats.collectFrom(population);
    }

    return {population.getBestFound(), stats};
}

Individual GeneticAlgorithm::crossover() const
{
    auto const parents = population.selectParents();

    std::vector<Individual> offspring;
    offspring.reserve(operators.size());

    for (auto const &op : operators)
        offspring.push_back(op(parents, params, rng));

    // A simple geometric acceptance criterion: select the best with some
    // probability. If not accepted, test the second best, etc.
    std::sort(offspring.begin(), offspring.end());

    for (auto &indiv : offspring)
        if (rng.randint(100) < params.config.selectProbability)
            return indiv;

    return offspring.back();  // fallback in case no offspring were selected
}

void GeneticAlgorithm::educate(Individual &indiv)
{
    localSearch(indiv, params.penaltyCapacity, params.penaltyTimeWarp);
    population.addIndividual(indiv);

    loadFeas.push_back(!indiv.hasExcessCapacity());
    loadFeas.pop_front();

    timeFeas.push_back(!indiv.hasTimeWarp());
    timeFeas.pop_front();

    if (!indiv.isFeasible()  // possibly repair if currently infeasible
        && rng.randint(100) < params.config.repairProbability)
    {
        localSearch(indiv,  // re-run, but penalise infeasibility more
                    params.config.repairBooster * params.penaltyCapacity,
                    params.config.repairBooster * params.penaltyTimeWarp);

        if (indiv.isFeasible())
            // TODO should we also register this individual in the load/time
            //  feasibility lists?
            population.addIndividual(indiv);
    }
}

void GeneticAlgorithm::updatePenalties()
{
    auto compute = [&](double currFeas, double currPenalty) {
        // +- 1 to ensure we do not get stuck at the same integer values.
        if (currFeas < 0.01 && params.config.feasBooster > 0)
            currPenalty = params.config.feasBooster * currPenalty + 1;
        else if (currFeas < params.config.targetFeasible - 0.05)
            currPenalty = params.config.penaltyIncrease * currPenalty + 1;
        else if (currFeas > params.config.targetFeasible + 0.05)
            currPenalty = params.config.penaltyDecrease * currPenalty - 1;

        // Bound the penalty term to avoid overflow in later cost computations.
        return static_cast<int>(std::max(std::min(currPenalty, 1000.), 1.));
    };

    double fracFeasLoad = std::accumulate(loadFeas.begin(), loadFeas.end(), 0.);
    fracFeasLoad /= static_cast<double>(loadFeas.size());

    double fracFeasTime = std::accumulate(timeFeas.begin(), timeFeas.end(), 0.);
    fracFeasTime /= static_cast<double>(timeFeas.size());

    params.penaltyCapacity = compute(fracFeasLoad, params.penaltyCapacity);
    params.penaltyTimeWarp = compute(fracFeasTime, params.penaltyTimeWarp);
}

GeneticAlgorithm::GeneticAlgorithm(Params &params,
                                   XorShift128 &rng,
                                   Population &population,
                                   LocalSearch &localSearch)
    : params(params),
      rng(rng),
      population(population),
      localSearch(localSearch),
      loadFeas(100, true),
      timeFeas(100, true)
{
}
