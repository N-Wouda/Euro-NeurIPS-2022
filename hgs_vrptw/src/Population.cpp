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

    // Find the individual's subpopulation
    auto &subpop = myIndividual->isFeasible() ? feasible : infeasible;

    for (Individual *other : subpop.individuals)
        myIndividual->brokenPairsDistance(other);

    // Identify the correct location in the population and insert the individual
    // TODO binsearch?
    int place = static_cast<int>(subpop.individuals.size());
    while (place > 0 && subpop.individuals[place - 1]->cost() > indiv.cost())
        place--;

    subpop.individuals.emplace(subpop.individuals.begin() + place,
                               myIndividual);
    subpop.fitness.emplace(subpop.fitness.begin() + place, 0);

    // Trigger a survivor selection if the maximum population size is exceeded
    size_t maxPopSize
        = params.config.minimumPopulationSize + params.config.generationSize;

    if (subpop.individuals.size() > maxPopSize)
    {
        // Remove duplicates before removing low fitness individuals
        while (subpop.individuals.size() > params.config.minimumPopulationSize)
            if (!removeDuplicate(subpop))
                break;

        while (subpop.individuals.size() > params.config.minimumPopulationSize)
        {
            updateBiasedFitness(subpop);
            removeWorstBiasedFitness(subpop);
        }
    }

    if (indiv.isFeasible() && indiv < bestSol)
        bestSol = indiv;
}

void Population::updateBiasedFitness(Subpopulation &subpop)
{
    if (subpop.individuals.size() == 1)
    {
        subpop.fitness[0] = 0;
        return;
    }

    // Ranking the individuals based on their diversity contribution (decreasing
    // order of broken pairs distance)
    std::vector<std::pair<double, size_t>> ranking;
    for (size_t idx = 0; idx != subpop.individuals.size(); idx++)
    {
        auto const dist
            = subpop.individuals[idx]->avgBrokenPairsDistanceClosest();
        ranking.emplace_back(dist, idx);
    }

    std::sort(ranking.begin(), ranking.end(), std::greater<>());

    auto const popSize = static_cast<double>(subpop.individuals.size());

    for (size_t idx = 0; idx != subpop.individuals.size(); idx++)
    {
        // Ranking the individuals based on the diversity rank and diversity
        // measure from 0 to 1
        double const divRank = idx / (popSize - 1);
        double const fitRank = ranking[idx].second / (popSize - 1);

        // Elite individuals cannot be smaller than population size
        // TODO can this be removed?
        if (subpop.individuals.size() <= params.config.nbElite)
            subpop.fitness[ranking[idx].second] = fitRank;
        // TODO can this be removed?
        else if (params.config.diversityWeight > 0)
            subpop.fitness[ranking[idx].second]
                = fitRank + params.config.diversityWeight * divRank;
        else
            subpop.fitness[ranking[idx].second]
                = fitRank + (1.0 - params.config.nbElite / popSize) * divRank;
    }
}

bool Population::removeDuplicate(Subpopulation &subpop)
{
    for (size_t idx = 0; idx != subpop.individuals.size(); idx++)
    {
        auto const *indiv = subpop.individuals[idx];

        if (indiv->hasClone())
        {
            subpop.individuals.erase(subpop.individuals.begin() + idx);
            subpop.fitness.erase(subpop.fitness.begin() + idx);
            delete indiv;

            return true;
        }
    }

    return false;
}

void Population::removeWorstBiasedFitness(Subpopulation &subpop)
{
    auto const worstFitness
        = std::max_element(subpop.fitness.begin(), subpop.fitness.end());
    auto const worstIdx = std::distance(subpop.fitness.begin(), worstFitness);
    auto const *worstIndividual = subpop.individuals[worstIdx];

    subpop.individuals.erase(subpop.individuals.begin() + worstIdx);
    subpop.fitness.erase(subpop.fitness.begin() + worstIdx);

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

    auto const feasSize = feasible.individuals.size();
    auto const popSize = feasSize + infeasible.individuals.size();

    // TODO refactor this with lambdas
    auto const idx1 = rng.randint(popSize);
    auto const *indiv1 = idx1 < feasSize
                             ? feasible.individuals[idx1]
                             : infeasible.individuals[idx1 - feasSize];
    auto const fitness1 = idx1 < feasSize ? feasible.fitness[idx1]
                                          : infeasible.fitness[idx1 - feasSize];

    auto const idx2 = rng.randint(popSize);
    auto const *indiv2 = idx2 < feasSize
                             ? feasible.individuals[idx2]
                             : infeasible.individuals[idx2 - feasSize];
    auto const fitness2 = idx2 < feasSize ? feasible.fitness[idx2]
                                          : infeasible.fitness[idx2 - feasSize];

    return fitness1 < fitness2 ? indiv1 : indiv2;
}

std::pair<Individual const *, Individual const *> Population::selectParents()
{
    updateBiasedFitness(feasible);
    updateBiasedFitness(infeasible);

    Individual const *par1 = getBinaryTournament();
    Individual const *par2 = getBinaryTournament();

    // TODO We need to check that par1 and par2 do not have identical routes.
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
