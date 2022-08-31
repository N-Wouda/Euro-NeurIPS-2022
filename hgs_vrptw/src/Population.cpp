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
    IndividualWrapper myIndividual = {std::make_unique<Individual>(indiv), 0};

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
    // Ranking the individuals based on their diversity contribution (decreasing
    // order of broken pairs distance)
    std::vector<std::pair<double, size_t>> diversity;
    for (size_t idx = 0; idx != subPop.size(); idx++)
    {
        auto const dist = subPop[idx].indiv->avgBrokenPairsDistanceClosest();
        diversity.emplace_back(dist, idx);
    }

    std::sort(diversity.begin(), diversity.end(), std::greater<>());

    auto const popSize = subPop.size();
    for (size_t divRank = 0; divRank != popSize; divRank++)
    {
        // Ranking the individuals based on the cost and diversity rank
        auto const costRank = diversity[divRank].second;
        auto const divWeight = std::max(0UL, popSize - params.config.nbElite);

        subPop[costRank].fitness = popSize * costRank + divWeight * divRank;
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
    auto const &worstFitness = std::max_element(
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

    auto const getIndividual = [&](auto const idx) -> auto &
    {
        return idx < feasSize ? feasible[idx] : infeasible[idx - feasSize];
    };

    auto const &wrapper1 = getIndividual(rng.randint(popSize));
    auto const &wrapper2 = getIndividual(rng.randint(popSize));

    return wrapper1.fitness < wrapper2.fitness ? wrapper1.indiv.get()
                                               : wrapper2.indiv.get();
}

std::pair<Individual const *, Individual const *> Population::selectParents()
{
    Individual const *par1 = getBinaryTournament();
    Individual const *par2 = getBinaryTournament();

    size_t numTries = 1;
    while ((par1 == par2 || *par1 == *par2)  // Try again a few more times
           && numTries++ < 10)               // if same parent
        par2 = getBinaryTournament();

    return std::make_pair(par1, par2);
}

Population::Population(Params &params, XorShift128 &rng)
    : params(params),
      rng(rng),
      bestSol(&params, &rng)  // random initial best solution
{
    generatePopulation(params.config.minimumPopulationSize);
}
