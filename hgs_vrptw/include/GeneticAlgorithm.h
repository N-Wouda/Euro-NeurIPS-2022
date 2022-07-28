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
#include <chrono>
#include <list>
#include <unordered_set>

// Class to run the genetic algorithm, which incorporates functionality of
// population management, doing crossovers and updating parameters.
class GeneticAlgorithm
{
    using clock = std::chrono::system_clock;
    using Parents = std::pair<Individual const *, Individual const *>;

    Params &params;            // Problem parameters
    XorShift128 &rng;          // Random number generator
    Population &population;    // Population
    LocalSearch &localSearch;  // Local search structure

    std::list<bool> loadFeas;  // load feasibility of recent individuals
    std::list<bool> timeFeas;  // time feasibility of recent individuals

    /**
     * Does one ordered crossover of the given parents. A randomly selected
     * subset of clients is taken from the first parent - the rest comes from
     * the second parent.
     */
    [[nodiscard]] Individual crossover(Parents const &parents) const;

    /**
     * Performs local search and adds the individual to the population. If the
     * individual is infeasible, with some probability we try to repair it and
     * add it if this succeeds.
     */
    void educate(Individual &indiv);

    /**
     * Updates the infeasibility penalties, based on the feasibility status of
     * the most recent individuals.
     */
    void updatePenalties();

public:
    /**
     * Runs the genetic algorithm until just after the passed-in time point.
     *
     * @param timePoint Time point in the future.
     * @return          Result object contained the best solution, and current
     *                  population composition.
     */
    Result runUntil(clock::time_point const &timePoint);

    GeneticAlgorithm(Params &params,
                     XorShift128 &rng,
                     Population &population,
                     LocalSearch &localSearch);
};

#endif
