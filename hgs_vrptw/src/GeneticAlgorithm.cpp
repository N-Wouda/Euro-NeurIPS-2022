#include "GeneticAlgorithm.h"

#include "Individual.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Result.h"
#include "Statistics.h"

#include <numeric>
#include <stdexcept>

Result GeneticAlgorithm::runUntil(clock::time_point const &timePoint)
{
    if (params.nbClients == 1)
        throw std::runtime_error("Cannot run genetic algorithm with one node.");

    Statistics stats;

    size_t iter = 0;
    size_t nbIterNonProd = 1;

    while (clock::now() < timePoint)
    {
        iter++;

        if (nbIterNonProd == params.config.nbIter)  // restart population after
        {                                           // this number of useless
            population.restart();                   // iterations
            nbIterNonProd = 1;
        }

        // Selection and crossover. Crossover is done several times, to
        // preselect a reasonable candidate solution before applying an
        // expensive educate step.
        auto parents = population.selectParents();
        auto offspring = crossover(parents);

        // TODO add other crossover types?
        for (size_t count = 1; count != params.config.nbCrossover; ++count)
        {
            parents = population.selectParents();
            auto cand = crossover(parents);

            if (cand.cost() < offspring.cost())
                offspring = cand;
        }

        auto const currBest = population.getBestFound().cost();
        educate(offspring);

        if (currBest > population.getBestFound().cost())
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

Individual GeneticAlgorithm::crossover(Parents const &parents) const
{
    auto const &tour1 = parents.first->getTour();
    auto const &tour2 = parents.second->getTour();

    std::vector<int> newTour(params.nbClients);
    std::vector<bool> copied(params.nbClients + 1, false);

    // [start, end] marks the clients selected from the first parent. The
    // remaining clients are taken from the second parent.
    size_t start = rng.randint(params.nbClients);
    size_t end = rng.randint(params.nbClients);
    while (end == start)
        end = rng.randint(params.nbClients);

    // Copy in place the elements from start to end (possibly "wrapping around"
    // the end of the array)
    size_t j = start;
    while (j % params.nbClients != (end + 1) % params.nbClients)
    {
        newTour[j % params.nbClients] = tour1[j % params.nbClients];
        copied[newTour[j % params.nbClients]] = true;  // mark as copied
        j++;
    }

    // Fill the remaining elements in the order given by the second parent
    for (int i = 1; i <= params.nbClients; i++)
    {
        int client = tour2[(end + i) % params.nbClients];

        if (!copied[client])  // copy now if not already in tour
        {
            newTour[j % params.nbClients] = client;
            j++;
        }
    }

    return {&params, newTour};
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
                    params.penaltyCapacity * 10.,
                    params.penaltyTimeWarp * 10.);

        if (indiv.isFeasible())
            // TODO should we also register this individual in the load/time
            //  feasibility lists?
            population.addIndividual(indiv);
    }
}

void GeneticAlgorithm::updatePenalties()
{
    auto compute = [&](double currFeas, double currPenalty) {
        if (currFeas < 0.01 && params.config.penaltyBooster > 0)
            currPenalty *= params.config.penaltyBooster;
        else if (currFeas < params.config.targetFeasible - 0.05)
            currPenalty *= params.config.penaltyIncrease;
        else if (currFeas > params.config.targetFeasible + 0.05)
            currPenalty *= params.config.penaltyDecrease;

        // Setting some bounds [0.1, 100000] to the penalty values for safety
        return std::max(std::min(currPenalty, 100000.), 0.1);
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
