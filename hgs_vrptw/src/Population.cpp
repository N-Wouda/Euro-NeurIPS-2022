#include "Population.h"

#include "Individual.h"
#include "Params.h"

#include <memory>
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
    IndividualWrapper myIndividual = {std::make_unique<Individual>(indiv), 0.};

    // Find the individual's subpopulation
    auto &subPop = myIndividual.indiv->isFeasible() ? feasible : infeasible;

    for (auto const &other : subPop)
        myIndividual.indiv->brokenPairsDistance(other.indiv.get());

    // Identify the correct location in the population and insert the individual
    // TODO binsearch?
    int place = static_cast<int>(subPop.size());
    while (place > 0 && subPop[place - 1].indiv->cost() > indiv.cost())
        place--;

    subPop.emplace(subPop.begin() + place, std::move(myIndividual));
    updateBiasedFitness(subPop);

    // Trigger a survivor selection if the maximum population size is exceeded
    size_t maxPopSize
        = params.config.minimumPopulationSize + params.config.generationSize;

    if (subPop.size() > maxPopSize)
    {
        // Remove duplicates before removing low fitness individuals
        while (subPop.size() > params.config.minimumPopulationSize)
            if (!removeDuplicate(subPop))
                break;

        while (subPop.size() > params.config.minimumPopulationSize)
        {
            updateBiasedFitness(subPop);
            removeWorstBiasedFitness(subPop);
        }
    }

    if (indiv.isFeasible() && indiv < bestSol)
        bestSol = indiv;
}

void Population::updateBiasedFitness(SubPopulation &subPop)
{
    if (subPop.size() == 1)
    {
        subPop[0].fitness = 0;
        return;
    }

    // Ranking the individuals based on their diversity contribution (decreasing
    // order of broken pairs distance)
    std::vector<std::pair<double, size_t>> ranking;
    for (size_t idx = 0; idx != subPop.size(); idx++)
    {
        auto const dist = subPop[idx].indiv->avgBrokenPairsDistanceClosest();
        ranking.emplace_back(dist, idx);
    }

    std::sort(ranking.begin(), ranking.end(), std::greater<>());

    auto const popSize = static_cast<double>(subPop.size());

    for (size_t idx = 0; idx != subPop.size(); idx++)
    {
        // Ranking the individuals based on the diversity rank and diversity
        // measure from 0 to 1
        double const divRank = idx / (popSize - 1);
        double const fitRank = ranking[idx].second / (popSize - 1);

        if (subPop.size() <= params.config.nbElite)
            subPop[ranking[idx].second].fitness = fitRank;
        else
            subPop[ranking[idx].second].fitness
                = fitRank + (1.0 - params.config.nbElite / popSize) * divRank;
    }
}

bool Population::removeDuplicate(SubPopulation &subPop)
{
    for (size_t idx = 0; idx != subPop.size(); idx++)
    {
        if (subPop[idx].indiv->hasClone())
        {
            subPop.erase(subPop.begin() + idx);
            return true;
        }
    }

    return false;
}

void Population::removeWorstBiasedFitness(SubPopulation &subPop)
{
    auto const worstFitness = std::max_element(
        subPop.begin(), subPop.end(), [](auto const &a, auto const &b) {
            return a.fitness < b.fitness;
        });
    auto const worstIdx = std::distance(subPop.begin(), worstFitness);
    subPop.erase(subPop.begin() + worstIdx);
}

void Population::restart()
{
    feasible.clear();
    infeasible.clear();
    generatePopulation(params.config.minimumPopulationSize);
}

Individual const *Population::getBinaryTournament()
{

    auto const feasSize = feasible.size();
    auto const popSize = feasSize + infeasible.size();

    auto const idx1 = rng.randint(popSize);
    auto const &indivWrapper1
        = idx1 < feasSize ? feasible[idx1] : infeasible[idx1 - feasSize];

    auto const idx2 = rng.randint(popSize);
    auto const &indivWrapper2
        = idx2 < feasSize ? feasible[idx2] : infeasible[idx2 - feasSize];

    return indivWrapper1.fitness < indivWrapper2.fitness
               ? indivWrapper1.indiv.get()
               : indivWrapper2.indiv.get();
}

std::pair<Individual const *, Individual const *> Population::selectParents()
{
    Individual const *par1 = getBinaryTournament();
    Individual const *par2 = getBinaryTournament();

    size_t numTries = 1;
    while ((par1 == par2 || *par1 == *par2) && numTries++ < 10)
        par2 = getBinaryTournament();   // Try again few more times
    return std::make_pair(par1, par2);  // if same parent
}

Population::Population(Params &params, XorShift128 &rng)
    : params(params),
      rng(rng),
      bestSol(&params, &rng)  // random initial best solution
{
    generatePopulation(params.config.minimumPopulationSize);
}

Population::~Population() {}
