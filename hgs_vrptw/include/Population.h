#ifndef POPULATION_H
#define POPULATION_H

#include "Individual.h"
#include "Params.h"
#include "Statistics.h"
#include "XorShift128.h"

#include <vector>

// Class representing the population of a genetic algorithm with do binary
// tournaments, update fitness values, etc.
class Population
{
    friend class Statistics;  // used to collect population statistics

    using Parents = std::pair<Individual const *, Individual const *>;

    Params &params;    // Problem parameters
    XorShift128 &rng;  // Random number generator

    struct Member
    {
        Individual *indiv;
        double fitness;
    };

    std::vector<Member> feasible;
    std::vector<Member> infeasible;

    Individual bestSol;

    // Evaluates the biased fitness of all individuals in the subpopulation
    void updateBiasedFitness(std::vector<Member> &subpop);

    // Removes a duplicate individual from the subpopulation if there exists
    // one. If there are multiple duplicate individuals, then the one with the
    // lowest index in ``individuals`` is removed first.
    bool removeDuplicate(std::vector<Member> &subpop);

    // Removes the worst individual in terms of biased fitness
    void removeWorstBiasedFitness(std::vector<Member> &subpop);

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
        auto const op = [](auto const &subs1, auto const &subs2) {
            return subs1.indiv->cost() < subs2.indiv->cost();
        };
        std::sort(feasible.begin(), feasible.end(), op);
        std::sort(infeasible.begin(), infeasible.end(), op);
    }

    // Selects two (if possible non-identical) parents by binary tournament
    Parents selectParents();

    /**
     * Returns the best feasible solution that was observed during
     * iteration.
     */
    [[nodiscard]] Individual const &getBestFound() const { return bestSol; }

    Population(Params &params, XorShift128 &rng);

    ~Population();
};

#endif
