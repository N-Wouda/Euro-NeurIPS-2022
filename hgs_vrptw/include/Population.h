/*MIT License

Original HGS-CVRP code: Copyright(c) 2020 Thibaut Vidal
Additional contributions: Copyright(c) 2022 ORTEC

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef POPULATION_H
#define POPULATION_H

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <list>
#include <vector>

// Class representing the population of a genetic algorithm with do binary
// tournaments, update fitness values, etc.
class Population
{
    using Parents = std::pair<Individual const *, Individual const *>;

    Params &params;                           // Problem parameters
    XorShift128 &rng;                         // Random number generator
    std::vector<Individual *> population;     // Population ordered asc. by cost
    std::vector<double> fitness;              // Population fitness
    std::list<bool> listFeasibilityLoad;      // load feas. recent individuals
    std::list<bool> listFeasibilityTimeWarp;  // time feas. recent individuals
    Individual bestSolutionOverall;           // best observed solution

    // Evaluates the biased fitness of all individuals in the population
    void updateBiasedFitness();

    // Removes the worst individual in terms of biased fitness
    void removeWorstBiasedFitness();

    // Generates a population of passed-in size
    void generatePopulation(size_t popSize);

    // Selects an individual by binary tournament
    Individual const *getBinaryTournament();

public:
    // Add an individual in the population (survivor selection is automatically
    // triggered whenever the population reaches its maximum size), and possibly
    // update the current best candidate.
    void addIndividual(Individual const &indiv, bool updateFeasible);

    // Cleans all solutions and generates a new initial population (only used
    // when running HGS until a time limit, in which case the algorithm restarts
    // until the time limit is reached)
    void restart();

    // Adaptation of the penalty parameters (this also updates the evaluations)
    void managePenalties();

    // Selects two (if possible non-identical) parents by binary tournament
    Parents selectParents();

    /**
     * Returns the best feasible solution that was observed during iteration.
     */
    [[nodiscard]] Individual const &getBestFound() const
    {
        return bestSolutionOverall;
    }

    Population(Params &params, XorShift128 &rng);

    ~Population();
};

#endif
