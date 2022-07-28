#ifndef POPULATION_H
#define POPULATION_H

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <vector>

// Class representing the population of a genetic algorithm with do binary
// tournaments, update fitness values, etc.
class Population
{
    using Parents = std::pair<Individual const *, Individual const *>;

    Params &params;    // Problem parameters
    XorShift128 &rng;  // Random number generator

    std::vector<Individual *> population;  // Population ordered asc. by cost
    std::vector<double> fitness;           // Population fitness
    Individual bestSol;                    // best observed solution

    // Evaluates the biased fitness of all individuals in the population
    void updateBiasedFitness();

    // Removes the worst individual in terms of biased fitness
    void removeWorstBiasedFitness();

    // Generates a population of passed-in size
    void generatePopulation(size_t popSize);

    // Selects an individual by binary tournament
    Individual const *getBinaryTournament();

public:
    // Add an individual in the population. Survivor selection is automatically
    // triggered whenever the population reaches its maximum size.
    void addIndividual(Individual const &indiv);

    // Cleans all solutions and generates a new initial population (only used
    // when running HGS until a time limit, in which case the algorithm restarts
    // until the time limit is reached)
    void restart();

    /**
     * Re-orders the population by cost.
     */
    void reorder()
    {
        std::sort(population.begin(),
                  population.end(),
                  [](auto const &indiv1, auto const &indiv2) {
                      return indiv1->cost() < indiv2->cost();
                  });
    }

    // Selects two (if possible non-identical) parents by binary tournament
    Parents selectParents();

    /**
     * Returns the best feasible solution that was observed during iteration.
     */
    [[nodiscard]] Individual const &getBestFound() const { return bestSol; }

    Population(Params &params, XorShift128 &rng);

    ~Population();
};

#endif
