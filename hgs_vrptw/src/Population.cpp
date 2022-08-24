#include "Population.h"

#include "Individual.h"
#include "Params.h"

#include <vector>

void Population::generatePopulation(size_t numToGenerate)
{
    for (size_t count = 0; count != numToGenerate; ++count)  // generate random
    {                                                        // individuals
        Individual randomIndiv(&params, &rng);
        addIndividual(randomIndiv);
    }
}

void Population::addIndividual(Individual const &indiv)
{
    // Create a copy of the individual and update the proximity structures
    // calculating inter-individual distances within its subpopulation
    auto *myIndividual = new Individual(indiv);

    // Find the individual's subpopulation based on feasibility
    auto &pop = myIndividual->isFeasible() ? feasible : infeasible;

    for (Individual *other : pop.individuals)
        myIndividual->brokenPairsDistance(other);

    // Identify the correct location in the population and insert the individual
    // TODO binsearch?
    int place = static_cast<int>(pop.individuals.size());
    while (place > 0 && pop.individuals[place - 1]->cost() > indiv.cost())
        place--;

    pop.individuals.emplace(pop.individuals.begin() + place, myIndividual);
    pop.fitness.emplace(pop.fitness.begin() + place, 0);

    // Trigger a survivor selection if the maximum population size is exceeded
    size_t maxPopSize
        = params.config.minimumPopulationSize + params.config.generationSize;

    if (pop.individuals.size() > maxPopSize)
    {
        // Remove duplicates before removing low fitness individuals
        while (pop.individuals.size() > params.config.minimumPopulationSize)
            if (!removeDuplicate(pop))
                break;

        while (pop.individuals.size() > params.config.minimumPopulationSize)
        {
            updateBiasedFitness(pop);
            removeWorstBiasedFitness(pop);
        }
    }

    if (indiv.isFeasible() && indiv < bestSol)
        bestSol = indiv;
}

void Population::updateBiasedFitness(Subpopulation &population)
{
    if (population.individuals.size() == 1)
    {
        population.fitness[0] = 0;
        return;
    }

    // Ranking the individuals based on their diversity contribution (decreasing
    // order of broken pairs distance)
    std::vector<std::pair<double, size_t>> ranking;
    for (size_t idx = 0; idx != population.individuals.size(); idx++)
    {
        auto const dist
            = population.individuals[idx]->avgBrokenPairsDistanceClosest();
        ranking.emplace_back(dist, idx);
    }

    std::sort(ranking.begin(), ranking.end(), std::greater<>());

    auto const popSize = static_cast<double>(population.individuals.size());

    for (size_t idx = 0; idx != population.individuals.size(); idx++)
    {
        // Ranking the individuals based on the diversity rank and diversity
        // measure from 0 to 1
        double const divRank = idx / (popSize - 1);
        double const fitRank = ranking[idx].second / (popSize - 1);

        // Elite individuals cannot be smaller than population size
        // TODO can this be removed?
        if (population.individuals.size() <= params.config.nbElite)
            population.fitness[ranking[idx].second] = fitRank;
        // TODO can this be removed?
        else if (params.config.diversityWeight > 0)
            population.fitness[ranking[idx].second]
                = fitRank + params.config.diversityWeight * divRank;
        else
            population.fitness[ranking[idx].second]
                = fitRank + (1.0 - params.config.nbElite / popSize) * divRank;
    }
}

bool Population::removeDuplicate(Subpopulation &population)
{
    for (size_t idx = 0; idx != population.individuals.size(); idx++)
    {
        auto const *indiv = population.individuals[idx];

        if (indiv->hasClone())
        {
            population.individuals.erase(population.individuals.begin() + idx);
            population.fitness.erase(population.fitness.begin() + idx);
            delete indiv;

            return true;
        }
    }

    return false;
}

void Population::removeWorstBiasedFitness(Subpopulation &population)
{
    auto const worstFitness = std::max_element(population.fitness.begin(),
                                               population.fitness.end());
    auto const worstIdx
        = std::distance(population.fitness.begin(), worstFitness);
    auto const *worstIndividual = population.individuals[worstIdx];

    population.individuals.erase(population.individuals.begin() + worstIdx);
    population.fitness.erase(population.fitness.begin() + worstIdx);

    delete worstIndividual;
}

void Population::restart()
{
    for (Individual *indiv : feasible.individuals)
        delete indiv;

    for (Individual *indiv : infeasible.individuals)
        delete indiv;

    // TODO can this be simplified?
    feasible.individuals.clear();
    feasible.fitness.clear();

    infeasible.individuals.clear();
    infeasible.fitness.clear();

    size_t const popSize = 4 * params.config.minimumPopulationSize;
    generatePopulation(popSize);
}

Individual const *Population::getBinaryTournament()
{
    // TODO do we really need to update here?
    updateBiasedFitness(feasible);
    updateBiasedFitness(infeasible);

    auto const feasSize = feasible.individuals.size();
    auto const infeasSize = infeasible.individuals.size();
    auto const popSize = feasSize + infeasSize;

    // TODO refactor this with lambdas
    auto const idx1 = rng.randint(popSize);
    auto const *indiv1 = (idx1 < feasSize)
                             ? feasible.individuals[idx1]
                             : infeasible.individuals[idx1 - feasSize];
    auto const fitness1 = (idx1 < feasSize)
                              ? feasible.fitness[idx1]
                              : infeasible.fitness[idx1 - feasSize];

    auto const idx2 = rng.randint(popSize);
    auto const *indiv2 = (idx2 < feasSize)
                             ? feasible.individuals[idx2]
                             : infeasible.individuals[idx2 - feasSize];
    auto const fitness2 = (idx2 < feasSize)
                              ? feasible.fitness[idx2]
                              : infeasible.fitness[idx2 - feasSize];

    return fitness1 < fitness2 ? indiv1 : indiv2;
}

std::pair<Individual const *, Individual const *> Population::selectParents()
{
    Individual const *par1 = getBinaryTournament();
    Individual const *par2 = getBinaryTournament();

    size_t numTries = 1;
    while (par1 == par2 && numTries++ < 10)  // try again a few more times if
        par2 = getBinaryTournament();        // same parent

    return std::make_pair(par1, par2);
}

Population::Population(Params &params, XorShift128 &rng)
    : params(params),
      rng(rng),
      bestSol(&params, &rng)  // random initial best solution
{
    // Generate a new population somewhat larger than the minimum size, so we
    // get a set of reasonable candidates early on.
    size_t const popSize = 4 * params.config.minimumPopulationSize;
    generatePopulation(popSize);
}

Population::~Population()
{
    for (auto &indiv : feasible.individuals)
        delete indiv;

    for (auto &indiv : infeasible.individuals)
        delete indiv;
}
