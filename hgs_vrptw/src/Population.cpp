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
    // calculating inter-individual distances
    auto *myIndividual = new Individual(indiv);

    for (Individual *other : population)
        myIndividual->brokenPairsDistance(other);

    // Identify the correct location in the population and insert the individual
    // TODO binsearch?
    int place = static_cast<int>(population.size());
    while (place > 0 && population[place - 1]->cost() > indiv.cost())
        place--;

    population.emplace(population.begin() + place, myIndividual);
    fitness.emplace(fitness.begin() + place, 0);

    // Trigger a survivor selection if the maximum population size is exceeded
    size_t maxPopSize
        = params.config.minimumPopulationSize + params.config.generationSize;

    if (population.size() > maxPopSize)
    {
        // Remove duplicates before removing low fitness individuals
        while (population.size() > params.config.minimumPopulationSize)
            if (!removeDuplicate())
                break;

        while (population.size() > params.config.minimumPopulationSize)
            removeWorstBiasedFitness();
    }
    if (indiv.isFeasible() && indiv < bestSol)
        bestSol = indiv;
}

void Population::updateBiasedFitness()
{
    if (population.size() == 1)
    {
        fitness[0] = 0;
        return;
    }

    // TODO should this be split by feas/infeas?

    // Ranking the individuals based on their diversity contribution (decreasing
    // order of broken pairs distance)
    std::vector<std::pair<double, size_t>> ranking;
    for (size_t idx = 0; idx != population.size(); idx++)
    {
        auto const dist
            = population[idx]->avgBrokenPairsDistance(params.config.nbClose);
        ranking.emplace_back(dist, idx);
    }

    std::sort(ranking.begin(), ranking.end(), std::greater<>());

    auto const popSize = static_cast<double>(population.size());

    for (size_t idx = 0; idx != population.size(); idx++)
    {
        // Ranking the individuals based on the diversity rank and diversity
        // measure from 0 to 1
        double const divRank = idx / (popSize - 1);
        double const fitRank = ranking[idx].second / (popSize - 1);

        // Elite individuals cannot be smaller than population size
        if (population.size() <= params.config.nbElite)
            fitness[ranking[idx].second] = fitRank;
        else if (params.config.diversityWeight > 0)
            fitness[ranking[idx].second]
                = fitRank + params.config.diversityWeight * divRank;
        else
            fitness[ranking[idx].second]
                = fitRank + (1.0 - params.config.nbElite / popSize) * divRank;
    }
}

bool Population::removeDuplicate()
{
    updateBiasedFitness();

    for (size_t idx = 0; idx != population.size(); idx++)
    {
        auto const *indiv = population[idx];
        auto const dist = indiv->avgBrokenPairsDistance(1);

        // An individual with near zero proximity indicates duplicity
        if (dist < 1e-7)
        {
            population.erase(population.begin() + idx);
            fitness.erase(fitness.begin() + idx);
            delete indiv;

            return true;
        }
    }
    return false;
}
void Population::removeWorstBiasedFitness()
{
    updateBiasedFitness();

    auto const worstFitness = std::max_element(fitness.begin(), fitness.end());
    auto const worstIdx = std::distance(fitness.begin(), worstFitness);
    auto const *worstIndividual = population[worstIdx];

    population.erase(population.begin() + worstIdx);
    fitness.erase(fitness.begin() + worstIdx);

    delete worstIndividual;
}

void Population::restart()
{
    for (Individual *indiv : population)
        delete indiv;

    population.clear();
    fitness.clear();

    size_t const popSize = 4 * params.config.minimumPopulationSize;
    generatePopulation(popSize);
}

Individual const *Population::getBinaryTournament()
{
    updateBiasedFitness();

    auto const idx1 = rng.randint(population.size());
    auto const *individual1 = population[idx1];
    auto const fitness1 = fitness[idx1];

    auto const idx2 = rng.randint(population.size());
    auto const *individual2 = population[idx2];
    auto const fitness2 = fitness[idx2];

    return fitness1 < fitness2 ? individual1 : individual2;
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
    for (auto &indiv : population)
        delete indiv;
}
