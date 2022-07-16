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
#include "LocalSearch.h"
#include "Params.h"
#include "XorShift128.h"

#include <list>
#include <vector>

// Class representing the population of a Genetic Algorithm with functionality
// to write information to files, do binary tournaments, update fitness values
// etc.
class Population
{
    using SubPopulation = std::vector<Individual *>;

    Params &params;            // Problem parameters
    XorShift128 &rng;          // Random number generator
    LocalSearch &localSearch;  // Local search structure

    // Feasible subpopulation kept ordered by increasing penalized cost
    SubPopulation feasibleSubpopulation;

    // Infeasible subpopulation kept ordered by increasing penalized cost
    SubPopulation infeasibleSubpopulation;

    // Load feasibility of the last 100 individuals generated by LS
    std::list<bool> listFeasibilityLoad;

    // Time warp feasibility of the last 100 individuals generated by LS
    std::list<bool> listFeasibilityTimeWarp;

    // Best solution found during the complete execution of the algorithm
    Individual bestSolutionOverall;

    // Evaluates the biased fitness of all individuals in the population
    void updateBiasedFitnesses(SubPopulation &pop) const;

    // Removes the worst individual in terms of biased fitness
    void removeWorstBiasedFitness(SubPopulation &subpop);

    // Performs local search and adds the individual. If the individual is
    // infeasible, with some probability we try to repair it and add it if this
    // succeeds.
    void doLocalSearchAndAddIndividual(Individual *indiv);

public:
    // Generates the population. Part of the population is generated randomly
    // and the other part using several construction heuristics. There is
    // variety in the individuals that are constructed using the construction
    // heuristics through the parameters used.
    void generatePopulation();

    // Add an individual in the population (survivor selection is automatically
    // triggered whenever the population reaches its maximum size) Returns TRUE
    // if a new best solution of the run has been found
    bool addIndividual(const Individual *indiv, bool updateFeasible);

    // Cleans all solutions and generates a new initial population (only used
    // when running HGS until a time limit, in which case the algorithm restarts
    // until the time limit is reached)
    void restart();

    // Adaptation of the penalty parameters (this also updates the evaluations)
    void managePenalties();

    // Selects an individal by binary tournament
    Individual *getBinaryTournament();

    // Selects two non-identical parents by binary tournament and return as a
    // pair
    std::pair<Individual *, Individual *>
    getNonIdenticalParentsBinaryTournament();

    [[nodiscard]] SubPopulation const &getFeasible() const
    {
        return feasibleSubpopulation;
    }

    [[nodiscard]] SubPopulation const &getInfeasible() const
    {
        return infeasibleSubpopulation;
    }

    Population(Params &params, XorShift128 &rng, LocalSearch &localSearch);

    ~Population();
};

#endif
