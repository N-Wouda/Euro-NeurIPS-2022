#include "Population.h"

#include "Individual.h"
#include "LocalSearch.h"
#include "Params.h"

#include <algorithm>
#include <cmath>
#include <list>
#include <vector>

void Population::doLocalSearchAndAddIndividual(Individual *indiv)
{
    // Do a Local Search
    localSearch->run(indiv, params->penaltyCapacity, params->penaltyTimeWarp);

    // Add an individual
    addIndividual(indiv, true);

    // With a certain probability, repair half of the solutions by increasing
    // the penalties for infeasibilities (w.r.t. capacities and time warps) in a
    // new Local Search in case of infeasibility
    if (!indiv->isFeasible()
        && params->rng() % 100 < (unsigned int)params->config.repairProbability)
    {
        localSearch->run(indiv,
                         params->penaltyCapacity * 10.,
                         params->penaltyTimeWarp * 10.);

        // Add the individual only when feasible
        if (indiv->isFeasible())
        {
            addIndividual(indiv, false);
        }
    }
}

void Population::generatePopulation()
{
    if (params->nbClients == 1)
    {
        // Quickly generate the one solution
        Individual randomIndiv(params);
        addIndividual(&randomIndiv, true);
        return;
    }
    // ------- The below parameters are configurable through command line
    // arguments ---------
    double fractionGeneratedNearest = params->config.fractionGeneratedNearest;
    double fractionGeneratedFurthest = params->config.fractionGeneratedFurthest;
    double fractionGeneratedSweep = params->config.fractionGeneratedSweep;
    double fractionGeneratedRandomly = params->config.fractionGeneratedRandomly;
    int minSweepFillPercentage = params->config.minSweepFillPercentage;
    int maxToleratedCapacityViolation
        = params->config.maxToleratedCapacityViolation;
    int maxToleratedTimeWarp = params->config.maxToleratedTimeWarp;
    double initialTimeWarpPenalty = params->config.initialTimeWarpPenalty;
    // ------- End of configurable parameters
    // -----------------------------------------------------

    // Generate same number of individuals as in original solution.
    int nofIndividuals = 4 * params->config.minimumPopulationSize;

    // TODO: Change next comment?
    // Note we actually set initial penalty in Params.cpp but by setting it here
    // we also reset it when resetting the population (probably not ideal but
    // test before changing)
    params->penaltyTimeWarp = initialTimeWarpPenalty;

    // Too low fill percentage may cause that not all clients are planned
    minSweepFillPercentage = std::max(minSweepFillPercentage, 30);
    int nofNearestIndividualsToGenerate
        = round(fractionGeneratedNearest * nofIndividuals);
    int nofFurthestIndividualsToGenerate
        = round(fractionGeneratedFurthest * nofIndividuals);
    int nofSweepIndividualsToGenerate
        = round(fractionGeneratedSweep * nofIndividuals);
    int nofRandomIndividualsToGenerate
        = round(fractionGeneratedRandomly * nofIndividuals);

    // Generate some individuals using the NEAREST construction heuristic
    for (int i = 0; i < nofNearestIndividualsToGenerate; i++)
    {
        if (params->isTimeLimitExceeded())
            throw std::runtime_error(
                "Time limit exceeded generating population.");

        // Create the first individual without violations
        int toleratedCapacityViolation
            = i == 0 ? 0 : params->rng() % (maxToleratedCapacityViolation + 1);
        int toleratedTimeWarp
            = i == 0 ? 0 : params->rng() % (maxToleratedTimeWarp + 1);
        Individual indiv(params, false);
        localSearch->constructIndividualWithSeedOrder(
            toleratedCapacityViolation, toleratedTimeWarp, false, &indiv);
        doLocalSearchAndAddIndividual(&indiv);
    }

    // Generate some individuals using the FURHEST construction heuristic
    for (int i = 0; i < nofFurthestIndividualsToGenerate; i++)
    {
        if (params->isTimeLimitExceeded())
            throw std::runtime_error(
                "Time limit exceeded generating population.");

        // Create the first individual without violations
        int toleratedCapacityViolation
            = i == 0 ? 0 : params->rng() % (maxToleratedCapacityViolation + 1);
        int toleratedTimeWarp
            = i == 0 ? 0 : params->rng() % (maxToleratedTimeWarp + 1);
        Individual indiv(params, false);
        localSearch->constructIndividualWithSeedOrder(
            toleratedCapacityViolation, toleratedTimeWarp, true, &indiv);
        doLocalSearchAndAddIndividual(&indiv);
    }

    // Generate some individuals using the SWEEP construction heuristic
    for (int i = 0; i < nofSweepIndividualsToGenerate; i++)
    {
        if (params->isTimeLimitExceeded())
            throw std::runtime_error(
                "Time limit exceeded generating population.");

        // Create the first individual without load restrictions
        int fillPercentage
            = i == 0 ? 100
                     : minSweepFillPercentage
                           + params->rng() % (100 - minSweepFillPercentage + 1);
        Individual indiv(params, false);
        localSearch->constructIndividualBySweep(fillPercentage, &indiv);
        doLocalSearchAndAddIndividual(&indiv);
    }

    // Generate some individuals using a RANDOM strategy
    for (int i = 0; i < nofRandomIndividualsToGenerate; i++)
    {
        if (params->isTimeLimitExceeded())
            throw std::runtime_error(
                "Time limit exceeded generating population.");

        Individual randomIndiv(params);
        doLocalSearchAndAddIndividual(&randomIndiv);
    }
}

bool Population::addIndividual(const Individual *indiv, bool updateFeasible)
{
    if (updateFeasible)  // update feasibility if needed
    {
        listFeasibilityLoad.push_back(indiv->costs.capacityExcess < MY_EPSILON);
        listFeasibilityTimeWarp.push_back(indiv->costs.timeWarp < MY_EPSILON);
        listFeasibilityLoad.pop_front();
        listFeasibilityTimeWarp.pop_front();
    }

    SubPopulation &pop  // where to insert?
        = indiv->isFeasible() ? feasibleSubpopulation : infeasibleSubpopulation;

    // Create a copy of the individual and update the proximity structures
    // calculating inter-individual distances
    auto *myIndividual = new Individual(*indiv);

    for (Individual *myIndividual2 : pop)
    {
        double myDistance = myIndividual->brokenPairsDistance(myIndividual2);
        myIndividual2->indivsPerProximity.insert({myDistance, myIndividual});
        myIndividual->indivsPerProximity.insert({myDistance, myIndividual2});
    }

    // Identify the correct location in the population and insert the individual
    // TODO binsearch?
    int place = static_cast<int>(pop.size());
    while (place > 0
           && pop[place - 1]->costs.penalizedCost
                  > indiv->costs.penalizedCost - MY_EPSILON)
    {
        place--;
    }
    pop.emplace(pop.begin() + place, myIndividual);

    // Trigger a survivor selection if the maximimum population size is exceeded
    size_t maxPopSize
        = params->config.minimumPopulationSize + params->config.generationSize;

    if (pop.size() > maxPopSize)
        while (pop.size() > params->config.minimumPopulationSize)
            removeWorstBiasedFitness(pop);

    // Track best solution
    if (indiv->isFeasible()
        && indiv->costs.penalizedCost
               < bestSolutionRestart.costs.penalizedCost - MY_EPSILON)
    {
        bestSolutionRestart = *indiv;
        if (indiv->costs.penalizedCost
            < bestSolutionOverall.costs.penalizedCost - MY_EPSILON)
        {
            bestSolutionOverall = *indiv;
            searchProgress.emplace_back(
                params->getElapsedTime(),
                bestSolutionOverall.costs.penalizedCost);
        }

        return true;
    }

    return false;
}

void Population::updateBiasedFitnesses(SubPopulation &pop)
{
    // Ranking the individuals based on their diversity contribution (decreasing
    // order of averageBrokenPairsDistanceClosest)
    std::vector<std::pair<double, size_t>> ranking;
    for (size_t i = 0; i < pop.size(); i++)
    {
        ranking.emplace_back(
            -pop[i]->averageBrokenPairsDistanceClosest(params->config.nbClose),
            i);
    }
    std::sort(ranking.begin(), ranking.end());

    // Updating the biased fitness values. If there is only one individual, its
    // biasedFitness is 0
    if (pop.size() == 1)
    {
        pop[0]->biasedFitness = 0;
    }
    else
    {
        // Loop over all individuals
        for (int i = 0; i < static_cast<int>(pop.size()); i++)
        {
            // Ranking the individuals based on the diversity rank and diversity
            // measure from 0 to 1
            double divRank
                = static_cast<double>(i) / static_cast<double>(pop.size() - 1);
            double fitRank
                = ranking[i].second / static_cast<double>(pop.size() - 1);

            // Elite individuals cannot be smaller than population size
            if (pop.size() <= params->config.nbElite)
            {
                pop[ranking[i].second]->biasedFitness = fitRank;
            }
            else if (params->config.diversityWeight > 0)
            {
                pop[ranking[i].second]->biasedFitness
                    = fitRank + params->config.diversityWeight * divRank;
            }
            else
            {
                pop[ranking[i].second]->biasedFitness
                    = fitRank
                      + (1.0
                         - static_cast<double>(params->config.nbElite)
                               / static_cast<double>(pop.size()))
                            * divRank;
            }
        }
    }
}

void Population::removeWorstBiasedFitness(SubPopulation &pop)
{
    // Update the fitness values
    updateBiasedFitnesses(pop);

    // Throw an error of the population has at most one individual
    if (pop.size() <= 1)
    {
        throw std::string(
            "Eliminating the best individual: this should not occur in HGS");
    }

    Individual *worstIndividual = nullptr;
    int worstIndividualPosition = -1;
    bool isWorstIndividualClone = false;
    double worstIndividualBiasedFitness = -1.e30;
    // Loop over all individuals and save the wordt individual
    for (int i = 1; i < static_cast<int>(pop.size()); i++)
    {
        // An averageBrokenPairsDistanceClosest equal to 0 indicates that a
        // clone exists
        bool isClone
            = (pop[i]->averageBrokenPairsDistanceClosest(1) < MY_EPSILON);
        if ((isClone && !isWorstIndividualClone)
            || (isClone == isWorstIndividualClone
                && pop[i]->biasedFitness > worstIndividualBiasedFitness))
        {
            worstIndividualBiasedFitness = pop[i]->biasedFitness;
            isWorstIndividualClone = isClone;
            worstIndividualPosition = i;
            worstIndividual = pop[i];
        }
    }

    // Remove the worst individual from the population
    pop.erase(pop.begin() + worstIndividualPosition);
    // Cleaning its distances from the other individuals in the population
    for (Individual *myIndividual2 : pop)
        myIndividual2->removeProximity(worstIndividual);

    // Freeing memory
    delete worstIndividual;
}

void Population::restart()
{
    for (Individual *indiv : feasibleSubpopulation)
        delete indiv;

    for (Individual *indiv : infeasibleSubpopulation)
        delete indiv;

    // Clear the pools of solutions and make a new empty individual as the best
    // solution after the restart
    feasibleSubpopulation.clear();
    infeasibleSubpopulation.clear();

    bestSolutionRestart = Individual();

    // Generate a new initial population
    generatePopulation();
}

void Population::managePenalties()
{
    // Setting some bounds [0.1,100000] to the penalty values for safety
    double fractionFeasibleLoad
        = static_cast<double>(std::count(
              listFeasibilityLoad.begin(), listFeasibilityLoad.end(), true))
          / static_cast<double>(listFeasibilityLoad.size());
    if (fractionFeasibleLoad <= 0.01 && params->config.penaltyBooster > 0.
        && params->penaltyCapacity < 100000.)
    {
        params->penaltyCapacity = std::min(
            params->penaltyCapacity * params->config.penaltyBooster, 100000.);
    }
    else if (fractionFeasibleLoad < params->config.targetFeasible - 0.05
             && params->penaltyCapacity < 100000.)
    {
        params->penaltyCapacity
            = std::min(params->penaltyCapacity * 1.2, 100000.);
    }
    else if (fractionFeasibleLoad > params->config.targetFeasible + 0.05
             && params->penaltyCapacity > 0.1)
    {
        params->penaltyCapacity = std::max(params->penaltyCapacity * 0.85, 0.1);
    }

    // Setting some bounds [0.1,100000] to the penalty values for safety
    double fractionFeasibleTimeWarp
        = static_cast<double>(std::count(listFeasibilityTimeWarp.begin(),
                                         listFeasibilityTimeWarp.end(),
                                         true))
          / static_cast<double>(listFeasibilityTimeWarp.size());
    if (fractionFeasibleTimeWarp <= 0.01 && params->config.penaltyBooster > 0.
        && params->penaltyTimeWarp < 100000.)
    {
        params->penaltyTimeWarp = std::min(
            params->penaltyTimeWarp * params->config.penaltyBooster, 100000.);
    }
    else if (fractionFeasibleTimeWarp < params->config.targetFeasible - 0.05
             && params->penaltyTimeWarp < 100000.)
    {
        params->penaltyTimeWarp
            = std::min(params->penaltyTimeWarp * 1.2, 100000.);
    }
    else if (fractionFeasibleTimeWarp > params->config.targetFeasible + 0.05
             && params->penaltyTimeWarp > 0.1)
    {
        params->penaltyTimeWarp = std::max(params->penaltyTimeWarp * 0.85, 0.1);
    }

    // Update the evaluations
    for (int i = 0; i < static_cast<int>(infeasibleSubpopulation.size()); i++)
    {
        infeasibleSubpopulation[i]->costs.penalizedCost
            = infeasibleSubpopulation[i]->costs.distance
              + params->penaltyCapacity
                    * infeasibleSubpopulation[i]->costs.capacityExcess
              + params->penaltyTimeWarp
                    * infeasibleSubpopulation[i]->costs.timeWarp;
    }

    // If needed, reorder the individuals in the infeasible subpopulation since
    // the penalty values have changed (simple bubble sort for the sake of
    // simplicity)
    for (int i = 0; i < static_cast<int>(infeasibleSubpopulation.size()); i++)
    {
        for (size_t j = 0; j < infeasibleSubpopulation.size() - i - 1; j++)
        {
            if (infeasibleSubpopulation[j]->costs.penalizedCost
                > infeasibleSubpopulation[j + 1]->costs.penalizedCost
                      + MY_EPSILON)
            {
                Individual *indiv = infeasibleSubpopulation[j];
                infeasibleSubpopulation[j] = infeasibleSubpopulation[j + 1];
                infeasibleSubpopulation[j + 1] = indiv;
            }
        }
    }
}

Individual *Population::getBinaryTournament()
{
    // TODO only compute updated biased fitness of selected individuals?
    updateBiasedFitnesses(feasibleSubpopulation);
    updateBiasedFitnesses(infeasibleSubpopulation);

    auto feasSize = feasibleSubpopulation.size();
    auto infeasSize = infeasibleSubpopulation.size();

    // Pick a first random number individual from the total population (of both
    // feasible and infeasible individuals)
    size_t idx1 = params->rng() % (feasSize + infeasSize);
    auto *individual1 = idx1 >= feasSize
                            ? infeasibleSubpopulation[idx1 - feasSize]
                            : feasibleSubpopulation[idx1];

    // Pick a second random number individual from the total population (of both
    // feasible and infeasible individuals)
    size_t idx2 = params->rng() % (feasSize + infeasSize);
    auto *individual2 = idx2 >= feasSize
                            ? infeasibleSubpopulation[idx2 - feasSize]
                            : feasibleSubpopulation[idx2];

    return individual1->biasedFitness < individual2->biasedFitness
               ? individual1
               : individual2;
}

std::pair<Individual *, Individual *>
Population::getNonIdenticalParentsBinaryTournament()
{
    // Pick two individual using a binary tournament
    Individual *par1 = getBinaryTournament();
    Individual *par2 = getBinaryTournament();

    // If same parent, try a few more times to get some diversity
    size_t numTries = 1;
    while (par1 == par2 && numTries++ < 10)
        par2 = getBinaryTournament();

    return std::make_pair(par1, par2);
}

Population::Population(Params *params, LocalSearch *localSearch)
    : params(params), localSearch(localSearch)
{
    // Create lists for the load feasibility of the last 100 individuals
    // generated by LS, where all feasibilities are set to true
    listFeasibilityLoad = std::list<bool>(100, true);
    listFeasibilityTimeWarp = std::list<bool>(100, true);

    // Generate a new population
    generatePopulation();
}

Population::~Population()
{
    for (auto &feas : feasibleSubpopulation)
        delete feas;

    for (auto &infeas : infeasibleSubpopulation)
        delete infeas;
}
