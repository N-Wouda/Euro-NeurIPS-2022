#include "Genetic.h"

#include "Individual.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Result.h"

#include <unordered_set>

Result Genetic::runUntil(timePoint const &timePoint)
{
    if (params.nbClients == 1)
        throw std::runtime_error("Cannot run genetic algorithm with one node.");

    int nbIterNonProd = 1;
    int iter = 0;

    while (std::chrono::system_clock::now() < timePoint)
    {
        iter++;

        if (nbIterNonProd == params.config.nbIter)  // restart population after
        {                                           // this number of useless
            population.restart();                   // iterations
            iter = 0;
            nbIterNonProd = 1;
        }

        /* SELECTION AND CROSSOVER */
        // First select parents using getNonIdenticalParentsBinaryTournament
        auto parents = population.getNonIdenticalParentsBinaryTournament();

        // Use the parents to create new individuals using OX and SREX
        Individual offspringOX = crossoverOX(parents);
        Individual offspringSREX = crossoverSREX(parents);

        // Return the best individual, based on penalizedCost
        auto offspring = offspringOX.cost() < offspringSREX.cost()
                             ? offspringOX
                             : offspringSREX;

        /* LOCAL SEARCH */
        // Run the Local Search on the new individual
        localSearch.run(
            &offspring, params.penaltyCapacity, params.penaltyTimeWarp);

        // Check if the new individual is the best feasible individual of the
        // population, based on penalizedCost
        bool isNewBest = population.addIndividual(&offspring, true);

        // In case of infeasibility, repair the individual with a certain
        // probability
        if (!offspring.isFeasible()
            && rng.randint(100) < (unsigned int)params.config.repairProbability)
        {
            // Run the Local Search again, but with penalties for
            // infeasibility multiplied by 10
            localSearch.run(&offspring,
                            params.penaltyCapacity * 10.,
                            params.penaltyTimeWarp * 10.);

            // If the individual is feasible now, check if it is the best
            // feasible individual of the population, based on penalizedCost and
            // add it to the population If the individual is not feasible now,
            // it is not added to the population
            if (offspring.isFeasible())
            {
                isNewBest = (population.addIndividual(&offspring, false)
                             || isNewBest);
            }
        }

        /* TRACKING THE NUMBER OF ITERATIONS SINCE LAST SOLUTION IMPROVEMENT */
        if (isNewBest)
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

    return {population.getBestFound()};
}

Individual Genetic::crossoverOX(Parents parents)
{
    // Picking the start and end of the crossover zone
    size_t start = rng.randint(params.nbClients);
    size_t end = rng.randint(params.nbClients);

    // If the start and end overlap, change the end of the crossover zone
    while (end == start)
        end = rng.randint(params.nbClients);

    auto const indiv1 = doOXcrossover(parents, start, end);
    auto const indiv2 = doOXcrossover(parents, start, end);

    return indiv1.cost() < indiv2.cost() ? indiv1 : indiv2;
}

Individual
Genetic::doOXcrossover(Parents parents, size_t start, size_t end) const
{
    auto const &tour1 = parents.first->getTour();
    auto const &tour2 = parents.second->getTour();

    std::vector<int> newTour(params.nbClients);

    // Tracks already inserted clients
    auto copied = std::vector<bool>(params.nbClients + 1, false);

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

Individual Genetic::crossoverSREX(Parents parents) const
{
    // Get the number of routes of both parents
    auto const &routesA = parents.first->getRoutes();
    auto const &routesB = parents.second->getRoutes();

    size_t nbRoutesA = parents.first->numRoutes();
    size_t nbRoutesB = parents.second->numRoutes();

    // Picking the start index of routes to replace of parent A
    // We like to replace routes with a large overlap of tasks, so we choose
    // adjacent routes (they are sorted on polar angle)
    size_t startA = rng.randint(nbRoutesA);
    size_t startB = startA < nbRoutesB ? startA : 0;
    size_t minRoutes = std::min(nbRoutesA, nbRoutesB);
    size_t nMovedRoutes = minRoutes == 1 ? 1 : rng.randint(minRoutes - 1) + 1;

    std::unordered_set<int> clientsInSelectedA;
    std::unordered_set<int> clientsInSelectedB;

    for (size_t r = 0; r < nMovedRoutes; r++)
    {
        clientsInSelectedA.insert(routesA[(startA + r) % nbRoutesA].begin(),
                                  routesA[(startA + r) % nbRoutesA].end());

        clientsInSelectedB.insert(routesB[(startB + r) % nbRoutesB].begin(),
                                  routesB[(startB + r) % nbRoutesB].end());
    }

    while (true)
    {
        // Difference for moving 'left' in parent A
        const int differenceALeft
            = static_cast<int>(std::count_if(
                  routesA[(startA - 1 + nbRoutesA) % nbRoutesA].begin(),
                  routesA[(startA - 1 + nbRoutesA) % nbRoutesA].end(),
                  [&clientsInSelectedB](int c) {
                      return clientsInSelectedB.find(c)
                             == clientsInSelectedB.end();
                  }))
              - static_cast<int>(std::count_if(
                  routesA[(startA + nMovedRoutes - 1) % nbRoutesA].begin(),
                  routesA[(startA + nMovedRoutes - 1) % nbRoutesA].end(),
                  [&clientsInSelectedB](int c) {
                      return clientsInSelectedB.find(c)
                             == clientsInSelectedB.end();
                  }));

        // Difference for moving 'right' in parent A
        const int differenceARight
            = static_cast<int>(std::count_if(
                  routesA[(startA + nMovedRoutes) % nbRoutesA].begin(),
                  routesA[(startA + nMovedRoutes) % nbRoutesA].end(),
                  [&clientsInSelectedB](int c) {
                      return clientsInSelectedB.find(c)
                             == clientsInSelectedB.end();
                  }))
              - static_cast<int>(
                  std::count_if(routesA[startA].begin(),
                                routesA[startA].end(),
                                [&clientsInSelectedB](int c) {
                                    return clientsInSelectedB.find(c)
                                           == clientsInSelectedB.end();
                                }));

        // Difference for moving 'left' in parent B
        const int differenceBLeft
            = static_cast<int>(std::count_if(
                  routesB[(startB - 1 + nMovedRoutes) % nbRoutesB].begin(),
                  routesB[(startB - 1 + nMovedRoutes) % nbRoutesB].end(),
                  [&clientsInSelectedA](int c) {
                      return clientsInSelectedA.find(c)
                             != clientsInSelectedA.end();
                  }))
              - static_cast<int>(std::count_if(
                  routesB[(startB - 1 + nbRoutesB) % nbRoutesB].begin(),
                  routesB[(startB - 1 + nbRoutesB) % nbRoutesB].end(),
                  [&clientsInSelectedA](int c) {
                      return clientsInSelectedA.find(c)
                             != clientsInSelectedA.end();
                  }));

        // Difference for moving 'right' in parent B
        const int differenceBRight
            = static_cast<int>(
                  std::count_if(routesB[startB].begin(),
                                routesB[startB].end(),
                                [&clientsInSelectedA](int c) {
                                    return clientsInSelectedA.find(c)
                                           != clientsInSelectedA.end();
                                }))
              - static_cast<int>(std::count_if(
                  routesB[(startB + nMovedRoutes) % nbRoutesB].begin(),
                  routesB[(startB + nMovedRoutes) % nbRoutesB].end(),
                  [&clientsInSelectedA](int c) {
                      return clientsInSelectedA.find(c)
                             != clientsInSelectedA.end();
                  }));

        const int bestDifference = std::min({differenceALeft,
                                             differenceARight,
                                             differenceBLeft,
                                             differenceBRight});

        if (bestDifference >= 0)
            break;

        if (bestDifference == differenceALeft)
        {
            for (int c : routesA[(startA + nMovedRoutes - 1) % nbRoutesA])
            {
                clientsInSelectedA.erase(clientsInSelectedA.find(c));
            }
            startA = (startA - 1 + nbRoutesA) % nbRoutesA;
            for (int c : routesA[startA])
            {
                clientsInSelectedA.insert(c);
            }
        }
        else if (bestDifference == differenceARight)
        {
            for (int c : routesA[startA])
            {
                clientsInSelectedA.erase(clientsInSelectedA.find(c));
            }
            startA = (startA + 1) % nbRoutesA;
            for (int c : routesA[(startA + nMovedRoutes - 1) % nbRoutesA])
            {
                clientsInSelectedA.insert(c);
            }
        }
        else if (bestDifference == differenceBLeft)
        {
            for (int c : routesB[(startB + nMovedRoutes - 1) % nbRoutesB])
            {
                clientsInSelectedB.erase(clientsInSelectedB.find(c));
            }
            startB = (startB - 1 + nbRoutesB) % nbRoutesB;
            for (int c : routesB[startB])
            {
                clientsInSelectedB.insert(c);
            }
        }
        else if (bestDifference == differenceBRight)
        {
            for (int c : routesB[startB])
            {
                clientsInSelectedB.erase(clientsInSelectedB.find(c));
            }
            startB = (startB + 1) % nbRoutesB;
            for (int c : routesB[(startB + nMovedRoutes - 1) % nbRoutesB])
            {
                clientsInSelectedB.insert(c);
            }
        }
    }

    // Identify differences between route sets
    std::unordered_set<int> clientsInSelectedANotB;
    std::copy_if(
        clientsInSelectedA.begin(),
        clientsInSelectedA.end(),
        std::inserter(clientsInSelectedANotB, clientsInSelectedANotB.end()),
        [&clientsInSelectedB](int c)
        { return clientsInSelectedB.find(c) == clientsInSelectedB.end(); });

    std::unordered_set<int> clientsInSelectedBNotA;
    std::copy_if(
        clientsInSelectedB.begin(),
        clientsInSelectedB.end(),
        std::inserter(clientsInSelectedBNotA, clientsInSelectedBNotA.end()),
        [&clientsInSelectedA](int c)
        { return clientsInSelectedA.find(c) == clientsInSelectedA.end(); });

    Individual indiv1(&params, &rng);
    Individual indiv2(&params, &rng);

    // Replace selected routes from parent A with routes from parent B
    for (size_t r = 0; r < nMovedRoutes; r++)
    {
        size_t indexA = (startA + r) % nbRoutesA;
        size_t indexB = (startB + r) % nbRoutesB;
        indiv1.routeChrom[indexA].clear();
        indiv2.routeChrom[indexA].clear();

        for (int c : routesB[indexB])
        {
            indiv1.routeChrom[indexA].push_back(c);

            if (clientsInSelectedBNotA.find(c) == clientsInSelectedBNotA.end())
                indiv2.routeChrom[indexA].push_back(c);
        }
    }

    // Move routes from parent A that are kept
    for (size_t r = nMovedRoutes; r < nbRoutesA; r++)
    {
        size_t indexA = (startA + r) % nbRoutesA;
        indiv1.routeChrom[indexA].clear();
        indiv2.routeChrom[indexA].clear();

        for (int c : routesA[indexA])
        {
            if (clientsInSelectedBNotA.find(c) == clientsInSelectedBNotA.end())
                indiv1.routeChrom[indexA].push_back(c);

            indiv2.routeChrom[indexA].push_back(c);
        }
    }

    // Delete any remaining routes that still lived in offspring
    for (size_t r = nbRoutesA; r < static_cast<size_t>(params.nbVehicles); r++)
    {
        indiv1.routeChrom[r].clear();
        indiv2.routeChrom[r].clear();
    }

    // Step 3: Insert unplanned clients (those that were in the removed routes
    // of A, but not the inserted routes of B)
    insertUnplannedTasks(&indiv1, clientsInSelectedANotB);
    insertUnplannedTasks(&indiv2, clientsInSelectedANotB);

    indiv1.evaluateCompleteCost();
    indiv2.evaluateCompleteCost();

    return indiv1.cost() < indiv2.cost() ? indiv1 : indiv2;
}

void Genetic::insertUnplannedTasks(
    Individual *offspring, std::unordered_set<int> const &unplannedTasks) const
{
    int newDistanceToInsert = INT_MAX;    // TODO:
    int newDistanceFromInsert = INT_MAX;  // TODO:
    int distanceDelta = INT_MAX;          // TODO:

    auto const &routes = offspring->getRoutes();

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
            if (routes[r].empty())
                break;

            newDistanceFromInsert = params.timeCost.get(c, routes[r][0]);
            if (earliestArrival + newDistanceFromInsert
                < params.cli[routes[r][0]].latestArrival)
            {
                distanceDelta = params.timeCost.get(0, c) + newDistanceToInsert
                                - params.timeCost.get(0, routes[r][0]);
                if (distanceDelta < bestDistance)
                {
                    bestDistance = distanceDelta;
                    bestLocation = {r, 0};
                }
            }

            for (int i = 1; i < static_cast<int>(routes[r].size()); i++)
            {
                newDistanceToInsert = params.timeCost.get(routes[r][i - 1], c);
                newDistanceFromInsert = params.timeCost.get(c, routes[r][i]);
                if (params.cli[routes[r][i - 1]].earliestArrival
                            + newDistanceToInsert
                        < latestArrival
                    && earliestArrival + newDistanceFromInsert
                           < params.cli[routes[r][i]].latestArrival)
                {
                    distanceDelta
                        = newDistanceToInsert + newDistanceFromInsert
                          - params.timeCost.get(routes[r][i - 1], routes[r][i]);
                    if (distanceDelta < bestDistance)
                    {
                        bestDistance = distanceDelta;
                        bestLocation = {r, i};
                    }
                }
            }

            newDistanceToInsert = params.timeCost.get(routes[r].back(), c);
            if (params.cli[routes[r].back()].earliestArrival
                    + newDistanceToInsert
                < latestArrival)
            {
                distanceDelta = newDistanceToInsert + params.timeCost.get(c, 0)
                                - params.timeCost.get(routes[r].back(), 0);
                if (distanceDelta < bestDistance)
                {
                    bestDistance = distanceDelta;
                    bestLocation = {r, static_cast<int>(routes[r].size())};
                }
            }
        }

        offspring->routeChrom[bestLocation.first].insert(
            offspring->routeChrom[bestLocation.first].begin()
                + bestLocation.second,
            c);
    }
}

Genetic::Genetic(Params &params,
                 XorShift128 &rng,
                 Population &population,
                 LocalSearch &localSearch)
    : params(params), rng(rng), population(population), localSearch(localSearch)
{
}
