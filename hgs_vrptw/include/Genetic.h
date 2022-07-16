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

#ifndef GENETIC_H
#define GENETIC_H

#include "Individual.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Result.h"
#include "XorShift128.h"

#include <array>
#include <unordered_set>

// Class to run the genetic algorithm, which incorporates functionality of
// population management, doing crossovers and updating parameters.
class Genetic
{
    using Parents = std::pair<Individual const *, Individual const *>;

    // The number of new potential offspring created from one individual
    static const int numberOfCandidateOffsprings = 4;

    Params &params;            // Problem parameters
    XorShift128 &rng;          // Random number generator
    Population &population;    // Population
    LocalSearch &localSearch;  // Local Search structure

    // Pointers for offspring to edit new offspring in place:
    // 0 and 1 are reserved for SREX, 2 and 3 are reserved for OX
    std::array<Individual *, numberOfCandidateOffsprings> candidateOffsprings;

    // Function to do two OX Crossovers for a pair of individuals (the two
    // parents) and return the best individual based on penalizedCost
    Individual *crossoverOX(Parents parents);

    // Function to do one (in place) OX Crossover for one individual 'result',
    // given the two parents and the beginning and end (inclusive) of the
    // crossover zone
    void doOXcrossover(Individual *result,
                       Parents parents,
                       size_t start,
                       size_t end);

    // Function to do two SREX Crossovers for a pair of individuals (the two
    // parents) and return the best individual based on penalizedCost
    Individual *crossoverSREX(Parents parents);

    // Insert unplanned tasks (those that were in the removed routes of A but
    // not the inserted routes of B or vice versa)
    void insertUnplannedTasks(Individual *offspring,
                              std::unordered_set<int> const &unplanned);

    // Function to do one OX and one SREX Crossover for a pair of individuals
    // (the two parents), and get the best result based on penalizedCost
    Individual *bestOfSREXAndOXCrossovers(Parents parents);

public:
    // Running the genetic algorithm until maxIterNonProd consecutive iterations
    // without improvement or a time limit (in seconds) is reached
    Result const run();

    // Constructor
    Genetic(Params &params,
            XorShift128 &rng,
            Population &population,
            LocalSearch &localSearch);

    // Destructor
    ~Genetic();
};

#endif
