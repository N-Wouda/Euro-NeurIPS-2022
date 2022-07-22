#include "GeneticAlgorithm.h"

#include "Individual.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Result.h"

Result GeneticAlgorithm::runUntil(timePoint const &timePoint)
{
    if (params.nbClients == 1)
        throw std::runtime_error("Cannot run genetic algorithm with one node.");

    size_t iter = 0;
    size_t nbIterNonProd = 1;

    while (std::chrono::system_clock::now() < timePoint)
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

        /* DIVERSIFICATION, PENALTY MANAGEMENT AND TRACES */
        // Update the penaltyTimeWarp and penaltyCapacity every 100 iterations
        if (iter % 100 == 0)
            population.managePenalties();

        /* OTHER PARAMETER CHANGES*/
        // Increase the nbGranular by growNbGranularSize (and set the correlated
        // vertices again) every certain number of iterations, if
        // growNbGranularSize is greater than 0
        if (iter > 0 && params.config.growNbGranularSize != 0
            && ((params.config.growNbGranularAfterIterations > 0
                 && iter % params.config.growNbGranularAfterIterations == 0)
                || (params.config.growNbGranularAfterNonImprovementIterations
                        > 0
                    && nbIterNonProd
                               % params.config
                                     .growNbGranularAfterNonImprovementIterations
                           == 0)))
        {
            // Note: changing nbGranular also changes how often the order is
            // reshuffled
            params.config.nbGranular += params.config.growNbGranularSize;
            params.setCorrelatedVertices();
        }

        // Increase the minimumPopulationSize by growPopulationSize every
        // certain number of iterations, if growPopulationSize is greater than 0
        if (iter > 0 && params.config.growPopulationSize != 0
            && ((params.config.growPopulationAfterIterations > 0
                 && iter % params.config.growPopulationAfterIterations == 0)
                || (params.config.growPopulationAfterNonImprovementIterations
                        > 0
                    && nbIterNonProd
                               % params.config
                                     .growPopulationAfterNonImprovementIterations
                           == 0)))
        {
            // This will automatically adjust after some iterations
            params.config.minimumPopulationSize
                += params.config.growPopulationSize;
        }
    }

    return {population.getBestFound(), iter};
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
    localSearch.run(indiv, params.penaltyCapacity, params.penaltyTimeWarp);
    population.addIndividual(indiv, true);

    if (!indiv.isFeasible()  // possibly repair if currently infeasible
        && rng.randint(100) < params.config.repairProbability)
    {
        localSearch.run(indiv,  // re-run, but penalise infeasibility more
                        params.penaltyCapacity * 10.,
                        params.penaltyTimeWarp * 10.);

        if (indiv.isFeasible())
            population.addIndividual(indiv, false);
    }
}

GeneticAlgorithm::GeneticAlgorithm(Params &params,
                                   XorShift128 &rng,
                                   Population &population,
                                   LocalSearch &localSearch)
    : params(params), rng(rng), population(population), localSearch(localSearch)
{
}
