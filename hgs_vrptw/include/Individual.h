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
    using Tour = std::vector<Client>;
    using Routes = std::vector<Tour>;

    size_t nbRoutes = 0;        // Number of routes
    size_t distance = 0;        // Total distance
    size_t capacityExcess = 0;  // Total excess load over all routes
    size_t timeWarp = 0;        // All route time warp of late arrivals

    // The other individuals in the population (cannot be the depot 0), ordered
    // by increasing proximity (the set container follows a natural ordering
    // based on the value of the first pair)
    std::multiset<std::pair<double, Individual *>> indivsPerProximity;

    Params *params;  // Problem parameters

    // Giant tour representing the individual: list of integers representing
    // clients (can not be the depot 0). Size is nbClients.
    Tour tourChrom;

    // For each vehicle, the associated sequence of deliveries (complete
    // solution). Size is nbVehicles, but quite a few routes are likely empty
    // - the numRoutes() member indicates the number of nonempty routes.
    Routes routeChrom;

    // Splits the tour chromosome into routes using the linear split algorithm
    void makeRoutes();

    // Evaluates this solution's objective value.
    void evaluateCompleteCost();

    /**
     * Returns a vector of [pred, succ] clients for each client (index).
     */
    [[nodiscard]] std::vector<std::pair<Client, Client>> getNeighbours() const;

public:
    /**
     * Returns this individual's objective (penalized cost).
     */
    [[nodiscard]] inline size_t cost() const
    {
        // clang-format off
        return distance
               + capacityExcess * params->penaltyCapacity
               + timeWarp * params->penaltyTimeWarp;
        // clang-format on
    }

    /**
     * Returns this individual's routing decisions.
     */
    [[nodiscard]] inline Routes const &getRoutes() const { return routeChrom; }

    /**
     * Returns this individual's giant tour chromosome.
     */
    [[nodiscard]] inline Tour const &getTour() const { return tourChrom; }

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

    // Computes and stores a distance measure with another individual, based on
    // the number of arcs that differ between two solutions.
    void brokenPairsDistance(Individual *other);

    // Returns the average distance of this individual to the individuals
    // nearest to it.
    [[nodiscard]] double avgBrokenPairsDistanceClosest() const;

    // Exports a solution in CVRPLib format (adds a final line with the
    // computational time).
    void exportCVRPLibFormat(std::string const &path, double time) const;

    Individual(Params *params, XorShift128 *rng);  // random individual

    Individual(Params *params, Tour tour);

    Individual(Params *params, Tour tour, Routes routes);

    ~Individual();
};

#endif
