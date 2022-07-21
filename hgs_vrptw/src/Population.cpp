#include "Population.h"

#include "Individual.h"
#include "Params.h"

#include <list>
#include <vector>

void Population::generatePopulation(size_t numToGenerate)
{
    for (size_t count = 0; count != numToGenerate; ++count)  // generate random
    {                                                        // individuals
        Individual randomIndiv(&params, &rng);
        addIndividual(randomIndiv, true);
    }
}

void Population::addIndividual(Individual const &indiv, bool updateFeasible)
{
    if (updateFeasible)  // update feasibility if needed
    {
        listFeasibilityLoad.push_back(!indiv.hasExcessCapacity());
        listFeasibilityTimeWarp.push_back(!indiv.hasTimeWarp());
        listFeasibilityLoad.pop_front();
        listFeasibilityTimeWarp.pop_front();
    }

    SubPopulation &pop  // where to insert?
        = indiv.isFeasible() ? feasibleSubpopulation : infeasibleSubpopulation;
    std::vector<double> &fitness
        = indiv.isFeasible() ? feasibleFitness : infeasibleFitness;

    // Create a copy of the individual and update the proximity structures
    // calculating inter-individual distances
    auto *myIndividual = new Individual(indiv);

    for (Individual *other : pop)
        myIndividual->brokenPairsDistance(other);

    // Identify the correct location in the population and insert the individual
    // TODO binsearch?
    int place = static_cast<int>(pop.size());
    while (place > 0 && pop[place - 1]->cost() > indiv.cost())
        place--;

    pop.emplace(pop.begin() + place, myIndividual);
    fitness.emplace(fitness.begin() + place, 0);

    // Trigger a survivor selection if the maximum population size is exceeded
    size_t maxPopSize
        = params.config.minimumPopulationSize + params.config.generationSize;

    if (pop.size() > maxPopSize)
        while (pop.size() > params.config.minimumPopulationSize)
            removeWorstBiasedFitness(pop, fitness);

    if (indiv.isFeasible() && indiv.cost() < bestSolutionOverall.cost())
        bestSolutionOverall = indiv;
}

void Population::updateBiasedFitnesses(SubPopulation &pop,
                                       std::vector<double> &fitness) const
{
    // Updating the biased fitness values. If there is only one individual, its
    // biasedFitness is 0
    if (pop.size() == 1)
    {
        fitness[0] = 0;
        return;
    }

    // Ranking the individuals based on their diversity contribution (decreasing
    // order of avgBrokenPairsDistanceClosest)
    std::vector<std::pair<double, size_t>> ranking;
    for (size_t idx = 0; idx != pop.size(); idx++)
    {
        auto const dist
            = pop[idx]->avgBrokenPairsDistanceClosest(params.config.nbClose);
        ranking.emplace_back(-dist, idx);
    }

    std::sort(ranking.begin(), ranking.end());

    auto const popSize = static_cast<double>(pop.size());

    for (size_t idx = 0; idx != pop.size(); idx++)
    {
        // Ranking the individuals based on the diversity rank and diversity
        // measure from 0 to 1
        double const divRank = idx / (popSize - 1);
        double const fitRank = ranking[idx].second / (popSize - 1);

        // Elite individuals cannot be smaller than population size
        if (pop.size() <= params.config.nbElite)
            fitness[ranking[idx].second] = fitRank;
        else if (params.config.diversityWeight > 0)
            fitness[ranking[idx].second]
                = fitRank + params.config.diversityWeight * divRank;
        else
            fitness[ranking[idx].second]
                = fitRank + (1.0 - params.config.nbElite / popSize) * divRank;
    }
}

void Population::removeWorstBiasedFitness(SubPopulation &pop,
                                          std::vector<double> &fitness)
{
    updateBiasedFitnesses(pop, fitness);

    if (pop.size() <= 1)
        throw std::runtime_error("Eliminating the best individual");

    auto worstFitness = std::max_element(fitness.begin(), fitness.end());
    auto worstIdx = std::distance(fitness.begin(), worstFitness);
    auto worstIndividual = pop[worstIdx];

    pop.erase(pop.begin() + worstIdx);
    fitness.erase(fitness.begin() + worstIdx);

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
    feasibleFitness.clear();

    infeasibleSubpopulation.clear();
    infeasibleFitness.clear();

    size_t const popSize = 4 * params.config.minimumPopulationSize;
    generatePopulation(popSize);
}

void Population::managePenalties()
{
    // Setting some bounds [0.1,100000] to the penalty values for safety
    double fractionFeasibleLoad
        = static_cast<double>(std::count(
              listFeasibilityLoad.begin(), listFeasibilityLoad.end(), true))
          / static_cast<double>(listFeasibilityLoad.size());
    if (fractionFeasibleLoad <= 0.01 && params.config.penaltyBooster > 0.
        && params.penaltyCapacity < 100000.)
    {
        params.penaltyCapacity = std::min(
            params.penaltyCapacity * params.config.penaltyBooster, 100000.);
    }
    else if (fractionFeasibleLoad < params.config.targetFeasible - 0.05
             && params.penaltyCapacity < 100000.)
    {
        params.penaltyCapacity
            = std::min(params.penaltyCapacity * 1.2, 100000.);
    }
    else if (fractionFeasibleLoad > params.config.targetFeasible + 0.05
             && params.penaltyCapacity > 0.1)
    {
        params.penaltyCapacity = std::max(params.penaltyCapacity * 0.85, 0.1);
    }

    // Setting some bounds [0.1,100000] to the penalty values for safety
    double fractionFeasibleTimeWarp
        = static_cast<double>(std::count(listFeasibilityTimeWarp.begin(),
                                         listFeasibilityTimeWarp.end(),
                                         true))
          / static_cast<double>(listFeasibilityTimeWarp.size());
    if (fractionFeasibleTimeWarp <= 0.01 && params.config.penaltyBooster > 0.
        && params.penaltyTimeWarp < 100000.)
    {
        params.penaltyTimeWarp = std::min(
            params.penaltyTimeWarp * params.config.penaltyBooster, 100000.);
    }
    else if (fractionFeasibleTimeWarp < params.config.targetFeasible - 0.05
             && params.penaltyTimeWarp < 100000.)
    {
        params.penaltyTimeWarp
            = std::min(params.penaltyTimeWarp * 1.2, 100000.);
    }
    else if (fractionFeasibleTimeWarp > params.config.targetFeasible + 0.05
             && params.penaltyTimeWarp > 0.1)
    {
        params.penaltyTimeWarp = std::max(params.penaltyTimeWarp * 0.85, 0.1);
    }

    for (auto &indiv : infeasibleSubpopulation)
        indiv->evaluateCompleteCost();

    // If needed, reorder the individuals in the infeasible subpopulation since
    // the penalty values have changed (simple bubble sort for the sake of
    // simplicity)
    for (int i = 0; i < static_cast<int>(infeasibleSubpopulation.size()); i++)
    {
        for (size_t j = 0; j < infeasibleSubpopulation.size() - i - 1; j++)
        {
            if (infeasibleSubpopulation[j]->cost()
                > infeasibleSubpopulation[j + 1]->cost() + MY_EPSILON)
            {
                Individual *indiv = infeasibleSubpopulation[j];
                infeasibleSubpopulation[j] = infeasibleSubpopulation[j + 1];
                infeasibleSubpopulation[j + 1] = indiv;
            }
        }
    }
}

Individual const *Population::getBinaryTournament()
{
    // TODO only compute updated biased fitness of selected individuals?
    updateBiasedFitnesses(feasibleSubpopulation, feasibleFitness);
    updateBiasedFitnesses(infeasibleSubpopulation, infeasibleFitness);

    auto feasSize = feasibleSubpopulation.size();
    auto infeasSize = infeasibleSubpopulation.size();

    // Pick a first random number individual from the total population (of both
    // feasible and infeasible individuals)
    size_t idx1 = rng.randint(feasSize + infeasSize);

    auto *individual1 = idx1 >= feasSize
                            ? infeasibleSubpopulation[idx1 - feasSize]
                            : feasibleSubpopulation[idx1];

    double const fitness1 = idx1 >= feasSize
                                ? infeasibleFitness[idx1 - feasSize]
                                : feasibleFitness[idx1];

    // Pick a second random number individual from the total population (of both
    // feasible and infeasible individuals)
    size_t idx2 = rng.randint(feasSize + infeasSize);

    auto *individual2 = idx2 >= feasSize
                            ? infeasibleSubpopulation[idx2 - feasSize]
                            : feasibleSubpopulation[idx2];

    double const fitness2 = idx2 >= feasSize
                                ? infeasibleFitness[idx2 - feasSize]
                                : feasibleFitness[idx2];

    return fitness1 < fitness2 ? individual1 : individual2;
}

std::pair<Individual const *, Individual const *> Population::selectParents()
{
    // Pick two individual using a binary tournament
    Individual const *par1 = getBinaryTournament();
    Individual const *par2 = getBinaryTournament();

    // If same parent, try a few more times to get some diversity
    size_t numTries = 1;
    while (par1 == par2 && numTries++ < 10)
        par2 = getBinaryTournament();

    return std::make_pair(par1, par2);
}

Population::Population(Params &params, XorShift128 &rng)
    : params(params),
      rng(rng),
      bestSolutionOverall(&params, &rng)  // random initial best solution
{
    // Create lists for the load feasibility of the last 100 individuals
    // generated by LS, where all feasibilities are set to true
    listFeasibilityLoad = std::list<bool>(100, true);
    listFeasibilityTimeWarp = std::list<bool>(100, true);

    // Generate a new population somewhat larger than the minimum size, so we
    // get a set of reasonable candidates early on.
    size_t const popSize = 4 * params.config.minimumPopulationSize;
    generatePopulation(popSize);
}

Population::~Population()
{
    for (auto &feas : feasibleSubpopulation)
        delete feas;

    for (auto &infeas : infeasibleSubpopulation)
        delete infeas;
}
