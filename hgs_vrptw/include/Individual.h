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
#include "XorShift128.h"

#include <set>
#include <string>
#include <vector>

// Object to represent one individual of a population.
class Individual
{
    using Client = int;
    using Clients = std::vector<Client>;

    double penalizedCost = 0.;  // Penalized cost of the solution
    size_t nbRoutes = 0;        // Number of routes
    size_t distance = 0;        // Total distance
    size_t waitTime = 0;        // All route wait time of early arrivals
    size_t capacityExcess = 0;  // Total excess load over all routes
    size_t timeWarp = 0;        // All route time warp of late arrivals

    Params *params;  // Problem parameters

    // Giant tour representing the individual: list of integers representing
    // clients (can not be the depot 0). Size is nbClients.
    Clients tourChrom;

    // For each vehicle, the associated sequence of deliveries (complete
    // solution). Size is nbVehicles, but quite a few routes are likely empty
    // - the numRoutes() member indicates the number of nonempty routes.
    std::vector<Clients> routeChrom;

    // The other individuals in the population (cannot be the depot 0), ordered
    // by increasing proximity (the set container follows a natural ordering
    // based on the value of the first pair)
    std::multiset<std::pair<double, Individual *>> indivsPerProximity;

    // Splits the tour chromosome into routes using the linear split algorithm
    void makeRoutes();

    /**
     * Returns a vector of [pred, succ] clients for each client (index).
     */
    [[nodiscard]] std::vector<std::pair<Client, Client>> getNeighbours() const;

public:
    /**
     * Returns this individual's objective (penalized cost).
     */
    [[nodiscard]] inline double cost() const { return penalizedCost; }

    /**
     * Returns this individual's routing decisions.
     */
    [[nodiscard]] inline std::vector<Clients> const &getRoutes() const
    {
        return routeChrom;
    }

    /**
     * Number of non-empty routes in this solution.
     */
    [[nodiscard]] inline size_t numRoutes() const { return nbRoutes; }

    /**
     * Returns this individual's giant tour chromosome.
     */
    [[nodiscard]] inline Clients const &getTour() const { return tourChrom; }

    /**
     * Returns true when this solution is feasible; false otherwise.
     */
    [[nodiscard]] inline bool isFeasible() const
    {
        return !hasExcessCapacity() && !hasTimeWarp();
    }

    /**
     * If true, then the route exceeds vehicle capacity.
     */
    [[nodiscard]] inline bool hasExcessCapacity() const
    {
        return capacityExcess > 0;
    }

    /**
     * If true, then the route violates time window constraints.
     */
    [[nodiscard]] inline bool hasTimeWarp() const { return timeWarp > 0; }

    // Evaluates this solution's objective value.
    void evaluateCompleteCost();

    // Removes the other from the proximity structure of this individual.
    void removeProximity(Individual *other);

    // Computes and stores a distance measure with another individual, based on
    // the number of arcs that differ between two solutions
    void brokenPairsDistance(Individual *other);

    // Returns the average distance of this individual with the nbClosest
    // individuals
    [[nodiscard]] double avgBrokenPairsDistanceClosest(size_t nbClosest) const;

    // Exports a solution in CVRPLib format (adds a final line with the
    // computational time)
    void exportCVRPLibFormat(std::string const &path, double time) const;

    bool operator==(Individual const &other) const;

    Individual(Params *params, XorShift128 *rng);  // random individual

    Individual(Params *params, Clients tour);  // from tour

    Individual(Params *params,  // from tour and routes
               Clients tour,
               std::vector<Clients> routes);
};

#endif
