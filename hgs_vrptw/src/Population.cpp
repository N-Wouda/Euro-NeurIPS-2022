#include "Population.h"

#include "Individual.h"
#include "Params.h"

#include <list>
#include <numeric>
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
        loadFeas.push_back(!indiv.hasExcessCapacity());
        timeFeas.push_back(!indiv.hasTimeWarp());
        loadFeas.pop_front();
        timeFeas.pop_front();
    }

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
        while (population.size() > params.config.minimumPopulationSize)
            removeWorstBiasedFitness();

    if (indiv.isFeasible() && indiv.cost() < bestSol.cost())
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
        auto const dist = population[idx]->avgBrokenPairsDistanceClosest();
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

void Population::managePenalties()
{
    auto compute = [&](double currFeas, double penalty) {
        if (currFeas <= 0.01 && params.config.penaltyBooster > 0)
            penalty *= params.config.penaltyBooster;
        else if (currFeas < params.config.targetFeasible - 0.05)
            penalty *= 1.2;
        else if (currFeas > params.config.targetFeasible + 0.05)
            penalty *= 0.85;

        // Setting some bounds [0.1, 100000] to the penalty values for safety
        return std::max(std::min(penalty, 100'000.), 0.1);
    };

    double fracFeasLoad = std::accumulate(loadFeas.begin(), loadFeas.end(), 0.);
    fracFeasLoad /= static_cast<double>(loadFeas.size());

    double fracFeasTime = std::accumulate(timeFeas.begin(), timeFeas.end(), 0.);
    fracFeasTime /= static_cast<double>(timeFeas.size());

    params.penaltyCapacity = compute(fracFeasLoad, params.penaltyCapacity);
    params.penaltyTimeWarp = compute(fracFeasTime, params.penaltyTimeWarp);

    // Reorder the population since the penalties have changed.
    std::sort(population.begin(),
              population.end(),
              [](auto const &indiv1, auto const &indiv2) {
                  return indiv1->cost() < indiv2->cost();
              });
}

Individual const *Population::getBinaryTournament()
{
    updateBiasedFitness();

    auto const idx1 = rng.randint(static_cast<int>(population.size()));
    auto const *individual1 = population[idx1];
    auto const fitness1 = fitness[idx1];

    auto const idx2 = rng.randint(static_cast<int>(population.size()));
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
    // Create lists for the load feasibility of the last 100 individuals
    // generated by LS, where all feasibilities are set to true
    loadFeas = std::list<bool>(100, true);
    timeFeas = std::list<bool>(100, true);

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
