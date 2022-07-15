#include "Genetic.h"

#include "Individual.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Result.h"

#include <unordered_set>

Result const Genetic::run()
{
    auto const maxIterNonProd = params.config.nbIter;
    auto const timeLimit = params.config.timeLimit;

    if (params.nbClients == 1)
        throw std::runtime_error("Cannot run genetic algorithm with one node.");

    // Do iterations of the Genetic Algorithm, until more then maxIterNonProd
    // consecutive iterations without improvement or a time limit (in seconds)
    // is reached
    int nbIterNonProd = 1;
    for (int nbIter = 0;
         nbIterNonProd <= maxIterNonProd && !params.isTimeLimitExceeded();
         nbIter++)
    {
        /* SELECTION AND CROSSOVER */
        // First select parents using getNonIdenticalParentsBinaryTournament
        // Then use the selected parents to create new individuals using OX and
        // SREX Finally select the best new individual based on
        // bestOfSREXAndOXCrossovers
        Individual *offspring = bestOfSREXAndOXCrossovers(
            population.getNonIdenticalParentsBinaryTournament());

        /* LOCAL SEARCH */
        // Run the Local Search on the new individual
        localSearch.run(
            offspring, params.penaltyCapacity, params.penaltyTimeWarp);
        // Check if the new individual is the best feasible individual of the
        // population, based on penalizedCost
        bool isNewBest = population.addIndividual(offspring, true);
        // In case of infeasibility, repair the individual with a certain
        // probability
        if (!offspring->isFeasible()
            && params.rng() % 100
                   < (unsigned int)params.config.repairProbability)
        {
            // Run the Local Search again, but with penalties for
            // infeasibilities multiplied by 10
            localSearch.run(offspring,
                            params.penaltyCapacity * 10.,
                            params.penaltyTimeWarp * 10.);
            // If the individual is feasible now, check if it is the best
            // feasible individual of the population, based on penalizedCost and
            // add it to the population If the individual is not feasible now,
            // it is not added to the population
            if (offspring->isFeasible())
            {
                isNewBest
                    = (population.addIndividual(offspring, false) || isNewBest);
            }
        }

        /* TRACKING THE NUMBER OF ITERATIONS SINCE LAST SOLUTION IMPROVEMENT */
        if (isNewBest)
            nbIterNonProd = 1;
        else
            nbIterNonProd++;

        /* DIVERSIFICATION, PENALTY MANAGEMENT AND TRACES */
        // Update the penaltyTimeWarp and penaltyCapacity every 100 iterations
        if (nbIter % 100 == 0)
            population.managePenalties();

        /* FOR TESTS INVOLVING SUCCESSIVE RUNS UNTIL A TIME LIMIT: WE RESET THE
         * ALGORITHM/POPULATION EACH TIME maxIterNonProd IS ATTAINED*/
        if (timeLimit != INT_MAX && nbIterNonProd == maxIterNonProd
            && params.config.doRepeatUntilTimeLimit)
        {
            population.restart();
            nbIterNonProd = 1;
        }

        /* OTHER PARAMETER CHANGES*/
        // Increase the nbGranular by growNbGranularSize (and set the correlated
        // vertices again) every certain number of iterations, if
        // growNbGranularSize is greater than 0
        if (nbIter > 0 && params.config.growNbGranularSize != 0
            && ((params.config.growNbGranularAfterIterations > 0
                 && nbIter % params.config.growNbGranularAfterIterations == 0)
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
            params.SetCorrelatedVertices();
        }

        // Increase the minimumPopulationSize by growPopulationSize every
        // certain number of iterations, if growPopulationSize is greater than 0
        if (nbIter > 0 && params.config.growPopulationSize != 0
            && ((params.config.growPopulationAfterIterations > 0
                 && nbIter % params.config.growPopulationAfterIterations == 0)
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

    return Result(population.getFeasible(), population.getInfeasible());
}

Individual *Genetic::crossoverOX(Parents parents)
{
    // Picking the start and end of the crossover zone
    size_t start = params.rng() % params.nbClients;
    size_t end = params.rng() % params.nbClients;

    // If the start and end overlap, change the end of the crossover zone
    while (end == start)
        end = params.rng() % params.nbClients;

    doOXcrossover(candidateOffsprings[2], parents, start, end);
    doOXcrossover(candidateOffsprings[3], parents, start, end);

    auto cand1Cost = candidateOffsprings[2]->costs.penalizedCost;
    auto cand2Cost = candidateOffsprings[3]->costs.penalizedCost;

    return cand1Cost < cand2Cost ? candidateOffsprings[2]
                                 : candidateOffsprings[3];
}

void Genetic::doOXcrossover(Individual *result,
                            Parents parents,
                            size_t start,
                            size_t end)
{
    // Frequency vector to track the clients which have been inserted already
    std::vector<bool> freqClient
        = std::vector<bool>(params.nbClients + 1, false);

    // Copy in place the elements from start to end (possibly "wrapping around"
    // the end of the array)
    size_t j = start;
    while (j % params.nbClients != (end + 1) % params.nbClients)
    {
        result->tourChrom[j % params.nbClients]
            = parents.first->tourChrom[j % params.nbClients];
        // Mark the client as copied
        freqClient[result->tourChrom[j % params.nbClients]] = true;
        j++;
    }

    // Fill the remaining elements in the order given by the second parent
    for (int i = 1; i <= params.nbClients; i++)
    {
        // Check if the next client is already copied in place
        int temp = parents.second->tourChrom[(end + i) % params.nbClients];
        // If the client is not yet copied in place, copy in place now
        if (!freqClient[temp])
        {
            result->tourChrom[j % params.nbClients] = temp;
            j++;
        }
    }

    result->makeRoutes();  // turn tour chromosome into routes
}

Individual *Genetic::crossoverSREX(Parents parents)
{
    // Get the number of routes of both parents
    int nOfRoutesA = parents.first->costs.nbRoutes;
    int nOfRoutesB = parents.second->costs.nbRoutes;

    // Picking the start index of routes to replace of parent A
    // We like to replace routes with a large overlap of tasks, so we choose
    // adjacent routes (they are sorted on polar angle)
    int startA = params.rng() % nOfRoutesA;
    int startB = startA < nOfRoutesB ? startA : 0;

    int nOfMovedRoutes
        = std::min(nOfRoutesA, nOfRoutesB) == 1
              ? 1
              : params.rng() % (std::min(nOfRoutesA - 1, nOfRoutesB - 1)) + 1;

    std::unordered_set<int> clientsInSelectedA;
    std::unordered_set<int> clientsInSelectedB;

    for (int r = 0; r < nOfMovedRoutes; r++)
    {
        clientsInSelectedA.insert(
            parents.first->routeChrom[(startA + r) % nOfRoutesA].begin(),
            parents.first->routeChrom[(startA + r) % nOfRoutesA].end());

        clientsInSelectedB.insert(
            parents.second->routeChrom[(startB + r) % nOfRoutesB].begin(),
            parents.second->routeChrom[(startB + r) % nOfRoutesB].end());
    }

    bool improved = true;
    while (improved)
    {
        // Difference for moving 'left' in parent A
        const int differenceALeft
            = static_cast<int>(std::count_if(
                  parents.first
                      ->routeChrom[(startA - 1 + nOfRoutesA) % nOfRoutesA]
                      .begin(),
                  parents.first
                      ->routeChrom[(startA - 1 + nOfRoutesA) % nOfRoutesA]
                      .end(),
                  [&clientsInSelectedB](int c) {
                      return clientsInSelectedB.find(c)
                             == clientsInSelectedB.end();
                  }))
              - static_cast<int>(std::count_if(
                  parents.first
                      ->routeChrom[(startA + nOfMovedRoutes - 1) % nOfRoutesA]
                      .begin(),
                  parents.first
                      ->routeChrom[(startA + nOfMovedRoutes - 1) % nOfRoutesA]
                      .end(),
                  [&clientsInSelectedB](int c) {
                      return clientsInSelectedB.find(c)
                             == clientsInSelectedB.end();
                  }));

        // Difference for moving 'right' in parent A
        const int differenceARight
            = static_cast<int>(std::count_if(
                  parents.first
                      ->routeChrom[(startA + nOfMovedRoutes) % nOfRoutesA]
                      .begin(),
                  parents.first
                      ->routeChrom[(startA + nOfMovedRoutes) % nOfRoutesA]
                      .end(),
                  [&clientsInSelectedB](int c) {
                      return clientsInSelectedB.find(c)
                             == clientsInSelectedB.end();
                  }))
              - static_cast<int>(
                  std::count_if(parents.first->routeChrom[startA].begin(),
                                parents.first->routeChrom[startA].end(),
                                [&clientsInSelectedB](int c) {
                                    return clientsInSelectedB.find(c)
                                           == clientsInSelectedB.end();
                                }));

        // Difference for moving 'left' in parent B
        const int differenceBLeft
            = static_cast<int>(std::count_if(
                  parents.second
                      ->routeChrom[(startB - 1 + nOfMovedRoutes) % nOfRoutesB]
                      .begin(),
                  parents.second
                      ->routeChrom[(startB - 1 + nOfMovedRoutes) % nOfRoutesB]
                      .end(),
                  [&clientsInSelectedA](int c) {
                      return clientsInSelectedA.find(c)
                             != clientsInSelectedA.end();
                  }))
              - static_cast<int>(std::count_if(
                  parents.second
                      ->routeChrom[(startB - 1 + nOfRoutesB) % nOfRoutesB]
                      .begin(),
                  parents.second
                      ->routeChrom[(startB - 1 + nOfRoutesB) % nOfRoutesB]
                      .end(),
                  [&clientsInSelectedA](int c) {
                      return clientsInSelectedA.find(c)
                             != clientsInSelectedA.end();
                  }));

        // Difference for moving 'right' in parent B
        const int differenceBRight
            = static_cast<int>(
                  std::count_if(parents.second->routeChrom[startB].begin(),
                                parents.second->routeChrom[startB].end(),
                                [&clientsInSelectedA](int c) {
                                    return clientsInSelectedA.find(c)
                                           != clientsInSelectedA.end();
                                }))
              - static_cast<int>(std::count_if(
                  parents.second
                      ->routeChrom[(startB + nOfMovedRoutes) % nOfRoutesB]
                      .begin(),
                  parents.second
                      ->routeChrom[(startB + nOfMovedRoutes) % nOfRoutesB]
                      .end(),
                  [&clientsInSelectedA](int c) {
                      return clientsInSelectedA.find(c)
                             != clientsInSelectedA.end();
                  }));

        const int bestDifference = std::min({differenceALeft,
                                             differenceARight,
                                             differenceBLeft,
                                             differenceBRight});

        if (bestDifference < 0)
        {
            if (bestDifference == differenceALeft)
            {
                for (int c :
                     parents.first->routeChrom[(startA + nOfMovedRoutes - 1)
                                               % nOfRoutesA])
                {
                    clientsInSelectedA.erase(clientsInSelectedA.find(c));
                }
                startA = (startA - 1 + nOfRoutesA) % nOfRoutesA;
                for (int c : parents.first->routeChrom[startA])
                {
                    clientsInSelectedA.insert(c);
                }
            }
            else if (bestDifference == differenceARight)
            {
                for (int c : parents.first->routeChrom[startA])
                {
                    clientsInSelectedA.erase(clientsInSelectedA.find(c));
                }
                startA = (startA + 1) % nOfRoutesA;
                for (int c :
                     parents.first->routeChrom[(startA + nOfMovedRoutes - 1)
                                               % nOfRoutesA])
                {
                    clientsInSelectedA.insert(c);
                }
            }
            else if (bestDifference == differenceBLeft)
            {
                for (int c :
                     parents.second->routeChrom[(startB + nOfMovedRoutes - 1)
                                                % nOfRoutesB])
                {
                    clientsInSelectedB.erase(clientsInSelectedB.find(c));
                }
                startB = (startB - 1 + nOfRoutesB) % nOfRoutesB;
                for (int c : parents.second->routeChrom[startB])
                {
                    clientsInSelectedB.insert(c);
                }
            }
            else if (bestDifference == differenceBRight)
            {
                for (int c : parents.second->routeChrom[startB])
                {
                    clientsInSelectedB.erase(clientsInSelectedB.find(c));
                }
                startB = (startB + 1) % nOfRoutesB;
                for (int c :
                     parents.second->routeChrom[(startB + nOfMovedRoutes - 1)
                                                % nOfRoutesB])
                {
                    clientsInSelectedB.insert(c);
                }
            }
        }
        else
            improved = false;
    }

    // Identify differences between route sets
    std::unordered_set<int> clientsInSelectedANotB;
    std::copy_if(
        clientsInSelectedA.begin(),
        clientsInSelectedA.end(),
        std::inserter(clientsInSelectedANotB, clientsInSelectedANotB.end()),
        [&clientsInSelectedB](int c) {
            return clientsInSelectedB.find(c) == clientsInSelectedB.end();
        });

    std::unordered_set<int> clientsInSelectedBNotA;
    std::copy_if(
        clientsInSelectedB.begin(),
        clientsInSelectedB.end(),
        std::inserter(clientsInSelectedBNotA, clientsInSelectedBNotA.end()),
        [&clientsInSelectedA](int c) {
            return clientsInSelectedA.find(c) == clientsInSelectedA.end();
        });

    // Replace selected routes from parent A with routes from parent B
    for (int r = 0; r < nOfMovedRoutes; r++)
    {
        int indexA = (startA + r) % nOfRoutesA;
        int indexB = (startB + r) % nOfRoutesB;
        candidateOffsprings[0]->routeChrom[indexA].clear();
        candidateOffsprings[1]->routeChrom[indexA].clear();

        for (int c : parents.second->routeChrom[indexB])
        {
            candidateOffsprings[0]->routeChrom[indexA].push_back(c);
            if (clientsInSelectedBNotA.find(c) == clientsInSelectedBNotA.end())
            {
                candidateOffsprings[1]->routeChrom[indexA].push_back(c);
            }
        }
    }

    // Move routes from parent A that are kept
    for (int r = nOfMovedRoutes; r < nOfRoutesA; r++)
    {
        int indexA = (startA + r) % nOfRoutesA;
        candidateOffsprings[0]->routeChrom[indexA].clear();
        candidateOffsprings[1]->routeChrom[indexA].clear();

        for (int c : parents.first->routeChrom[indexA])
        {
            if (clientsInSelectedBNotA.find(c) == clientsInSelectedBNotA.end())
            {
                candidateOffsprings[0]->routeChrom[indexA].push_back(c);
            }
            candidateOffsprings[1]->routeChrom[indexA].push_back(c);
        }
    }

    // Delete any remaining routes that still lived in offspring
    for (int r = nOfRoutesA; r < params.nbVehicles; r++)
    {
        candidateOffsprings[0]->routeChrom[r].clear();
        candidateOffsprings[1]->routeChrom[r].clear();
    }

    // Step 3: Insert unplanned clients (those that were in the removed routes
    // of A but not the inserted routes of B)
    insertUnplannedTasks(candidateOffsprings[0], clientsInSelectedANotB);
    insertUnplannedTasks(candidateOffsprings[1], clientsInSelectedANotB);

    candidateOffsprings[0]->evaluateCompleteCost();
    candidateOffsprings[1]->evaluateCompleteCost();

    return candidateOffsprings[0]->costs.penalizedCost
                   < candidateOffsprings[1]->costs.penalizedCost
               ? candidateOffsprings[0]
               : candidateOffsprings[1];
}

void Genetic::insertUnplannedTasks(
    Individual *offspring, std::unordered_set<int> const &unplannedTasks)
{
    // Initialize some variables
    int newDistanceToInsert = INT_MAX;    // TODO:
    int newDistanceFromInsert = INT_MAX;  // TODO:
    int distanceDelta = INT_MAX;          // TODO:

    // Loop over all unplannedTasks
    for (int c : unplannedTasks)
    {
        // Get the earliest and laster possible arrival at the client
        int earliestArrival = params.cli[c].earliestArrival;
        int latestArrival = params.cli[c].latestArrival;

        int bestDistance = INT_MAX;
        std::pair<int, int> bestLocation;

        // Loop over all routes
        for (int r = 0; r < params.nbVehicles; r++)
        {
            // Go to the next route if this route is empty
            if (offspring->routeChrom[r].empty())
            {
                continue;
            }

            newDistanceFromInsert
                = params.timeCost.get(c, offspring->routeChrom[r][0]);
            if (earliestArrival + newDistanceFromInsert
                < params.cli[offspring->routeChrom[r][0]].latestArrival)
            {
                distanceDelta
                    = params.timeCost.get(0, c) + newDistanceToInsert
                      - params.timeCost.get(0, offspring->routeChrom[r][0]);
                if (distanceDelta < bestDistance)
                {
                    bestDistance = distanceDelta;
                    bestLocation = {r, 0};
                }
            }

            for (int i = 1;
                 i < static_cast<int>(offspring->routeChrom[r].size());
                 i++)
            {
                newDistanceToInsert
                    = params.timeCost.get(offspring->routeChrom[r][i - 1], c);
                newDistanceFromInsert
                    = params.timeCost.get(c, offspring->routeChrom[r][i]);
                if (params.cli[offspring->routeChrom[r][i - 1]].earliestArrival
                            + newDistanceToInsert
                        < latestArrival
                    && earliestArrival + newDistanceFromInsert
                           < params.cli[offspring->routeChrom[r][i]]
                                 .latestArrival)
                {
                    distanceDelta
                        = newDistanceToInsert + newDistanceFromInsert
                          - params.timeCost.get(offspring->routeChrom[r][i - 1],
                                                offspring->routeChrom[r][i]);
                    if (distanceDelta < bestDistance)
                    {
                        bestDistance = distanceDelta;
                        bestLocation = {r, i};
                    }
                }
            }

            newDistanceToInsert
                = params.timeCost.get(offspring->routeChrom[r].back(), c);
            if (params.cli[offspring->routeChrom[r].back()].earliestArrival
                    + newDistanceToInsert
                < latestArrival)
            {
                distanceDelta
                    = newDistanceToInsert + params.timeCost.get(c, 0)
                      - params.timeCost.get(offspring->routeChrom[r].back(), 0);
                if (distanceDelta < bestDistance)
                {
                    bestDistance = distanceDelta;
                    bestLocation = {
                        r, static_cast<int>(offspring->routeChrom[r].size())};
                }
            }
        }

        offspring->routeChrom[bestLocation.first].insert(
            offspring->routeChrom[bestLocation.first].begin()
                + bestLocation.second,
            c);
    }
}

Individual *Genetic::bestOfSREXAndOXCrossovers(Parents parents)
{
    // Create two individuals, one with OX and one with SREX
    Individual *offspringOX = crossoverOX(parents);
    Individual *offspringSREX = crossoverSREX(parents);

    // Return the best individual, based on penalizedCost
    return offspringOX->costs.penalizedCost < offspringSREX->costs.penalizedCost
               ? offspringOX
               : offspringSREX;
}

Genetic::Genetic(Params &params,
                 Population &population,
                 LocalSearch &localSearch)
    : params(params), population(population), localSearch(localSearch)
{
    // After initializing the parameters of the Genetic object, also generate
    // new individuals in the array candidateOffsprings
    std::generate(candidateOffsprings.begin(), candidateOffsprings.end(), [&] {
        return new Individual(&params);
    });
}

Genetic::~Genetic()
{
    for (Individual *candidateOffspring : candidateOffsprings)
        delete candidateOffspring;
}
