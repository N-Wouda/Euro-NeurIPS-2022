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

#ifndef INDIVIDUAL_H
#define INDIVIDUAL_H

#include "Params.h"

#include <set>
#include <string>
#include <vector>

// Object to store all relevant information that may be needed to calculate some
// cost corresponding to a solution
struct CostSol
{
    double penalizedCost = 0.;  // Penalized cost of the solution
    int nbRoutes = 0;           // Number of routes
    int distance = 0;           // Total Distance
    int capacityExcess = 0;     // Total excess load over all routes
    int waitTime = 0;  // Total wait time (time to wait to meet earliest
                       // possible arrival) over all routes
    int timeWarp = 0;  // Total time warp (going back in time to meet latest
                       // possible arrival) over all routes
};

// Object to represent one individual/solution of a population.
class Individual
{
    using Client = int;
    using Clients = std::vector<Client>;

public:
    // TODO make data members private

    Params *params;        // Problem parameters
    CostSol costs;         // Information on the cost of the solution
    bool isFeasible;       // Feasibility status of the individual
    double biasedFitness;  // Biased fitness of the solution

    // Giant tour representing the individual: list of integers representing
    // clients (can not be the depot 0). Size is nbClients.
    Clients tourChrom;

    // For each vehicle, the associated sequence of deliveries (complete
    // solution). Size is nbVehicles. Routes are stored starting index
    // maxVehicles - 1, so the first indices will likely be empty.
    std::vector<Clients> routeChrom;

    // For each node, the successor in the solution (can be the depot 0). Size
    // is nbClients + 1.
    Clients successors;

    // For each node, the predecessor in the solution (can be the depot 0). Size
    // is nbClients + 1.
    Clients predecessors;

    // The other individuals in the population (cannot be the depot 0), ordered
    // by increasing proximity (the set container follows a natural ordering
    // based on the value of the first pair)
    std::multiset<std::pair<double, Individual *>> indivsPerProximity;

    [[nodiscard]] std::vector<Clients> const &getRoutes() const
    {
        return routeChrom;
    }

    [[nodiscard]] Clients const &getTour() const { return tourChrom; }

    // Measuring cost of a solution from the information of routeChrom
    void evaluateCompleteCost();

    // Removes the other from the proximity structure of this individual.
    void removeProximity(Individual *other);

    // Distance measure with another individual, based on the number of arcs
    // that differ between two solutions
    double brokenPairsDistance(Individual *other);

    // Returns the average distance of this individual with the nbClosest
    // individuals
    [[nodiscard]] double
    averageBrokenPairsDistanceClosest(size_t nbClosest) const;

    // Exports a solution in CVRPLib format (adds a final line with the
    // computational time)
    void exportCVRPLibFormat(std::string const &path) const;

    bool operator==(Individual const &other) const;

    // Constructor: create a random individual
    explicit Individual(Params *params, bool initializeChromTAndShuffle = true);

    // Constructor: create an empty individual
    Individual();
};

#endif
