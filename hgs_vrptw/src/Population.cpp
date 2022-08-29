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

    for (Member &other : subpop)
        myIndividual->brokenPairsDistance(other.indiv);

    // Identify the correct location in the population and insert the individual
    // TODO binsearch?
    int place = static_cast<int>(subpop.size());
    while (place > 0 && subpop[place - 1].indiv->cost() > indiv.cost())
        place--;

    subpop.emplace(subpop.begin() + place, Member{myIndividual, 0.});

    // Trigger a survivor selection if the maximum population size is exceeded
    size_t maxPopSize
        = params.config.minimumPopulationSize + params.config.generationSize;

    if (subpop.size() > maxPopSize)
    {
        // Remove duplicates before removing low fitness individuals
        while (subpop.size() > params.config.minimumPopulationSize)
            if (!removeDuplicate(subpop))
                break;

        while (subpop.size() > params.config.minimumPopulationSize)
        {
            updateBiasedFitness(subpop);
            removeWorstBiasedFitness(subpop);
        }
    }

    if (indiv.isFeasible() && indiv < bestSol)
        bestSol = indiv;
}

void Population::updateBiasedFitness(std::vector<Member> &subpop)
{

    if (subpop.size() == 1)
    {
        subpop[0].fitness = 0;
        return;
    }

    // Ranking the individuals based on their diversity contribution (decreasing
    // order of broken pairs distance)
    std::vector<std::pair<double, size_t>> ranking;
    for (size_t idx = 0; idx != subpop.size(); idx++)
    {
        auto const dist = subpop[idx].indiv->avgBrokenPairsDistanceClosest();
        ranking.emplace_back(dist, idx);
    }

    std::sort(ranking.begin(), ranking.end(), std::greater<>());

    auto const popSize = static_cast<double>(subpop.size());

    for (size_t idx = 0; idx != subpop.size(); idx++)
    {
        // Ranking the individuals based on the diversity rank and diversity
        // measure from 0 to 1
        double const divRank = idx / (popSize - 1);
        double const fitRank = ranking[idx].second / (popSize - 1);

        if (subpop.size() <= params.config.nbElite)
            subpop[ranking[idx].second].fitness = fitRank;
        else
            subpop[ranking[idx].second].fitness
                = fitRank + (1.0 - params.config.nbElite / popSize) * divRank;
    }
}

bool Population::removeDuplicate(std::vector<Member> &subpop)
{
    for (size_t idx = 0; idx != subpop.size(); idx++)
    {
        auto const *indiv = subpop[idx].indiv;

        if (indiv->hasClone())
        {
            subpop.erase(subpop.begin() + idx);
            delete indiv;

            return true;
        }
    }

    return false;
}

void Population::removeWorstBiasedFitness(std::vector<Member> &subpop)
{
    auto const worstFitness = std::max_element(
        subpop.begin(), subpop.end(), [](const Member &a, const Member &b) {
            return a.fitness < b.fitness;
        });
    auto const worstIdx = std::distance(subpop.begin(), worstFitness);
    auto const *worstIndividual = subpop[worstIdx].indiv;

    subpop.erase(subpop.begin() + worstIdx);
    delete worstIndividual;
}

void Population::restart()
{
    for (Member &member : feasible)
        delete member.indiv;

    for (Member &member : infeasible)
        delete member.indiv;

    feasible.clear();
    infeasible.clear();

    size_t const popSize = 4 * params.config.minimumPopulationSize;
    generatePopulation(popSize);
}

Individual const *Population::getBinaryTournament()
{

    auto const feasSize = feasible.size();
    auto const popSize = feasSize + infeasible.size();

    auto const idx1 = rng.randint(popSize);
    auto const member1
        = idx1 < feasSize ? feasible[idx1] : infeasible[idx1 - feasSize];

    auto const idx2 = rng.randint(popSize);
    auto const member2
        = idx2 < feasSize ? feasible[idx2] : infeasible[idx2 - feasSize];

    return member1.fitness < member2.fitness ? member1.indiv : member2.indiv;
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
    for (auto &member : feasible)
        delete member.indiv;

    for (auto &member : infeasible)
        delete member.indiv;
}
