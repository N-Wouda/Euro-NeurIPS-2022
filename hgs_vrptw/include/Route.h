#ifndef HGS_VRPTW_ROUTE_H
#define HGS_VRPTW_ROUTE_H

#include "CircleSector.h"
#include "Node.h"
#include "Penalties.h"
#include "TimeWindowSegment.h"

#include <array>
#include <bit>
#include <cassert>
#include <iosfwd>

class Route
{
    constexpr static size_t jumpDistance = 4;

    // Vector of jumps. Each element is a time window segment of the route
    // between the indexed node and the node jumpDistance ahead in the route.
    // This lets us jump to the next node in the route a given jump distance
    // away. Very useful for speeding up time window calculations.
    std::vector<TimeWindowSegment> jumps;

    CircleSector sector;        // Circle sector of the route's clients
    std::vector<Node *> nodes;  // List of nodes (in order) in this solution.

    // Populates the nodes vector.
    void setupNodes();

    // Sets the angle and sector data.
    void setupSector();

    // Sets forward node time windows.
    void setupRouteTimeWindows();

public:  // TODO make fields private
    Params const *params;

    int idx;             // Route index
    Node *depot;         // Pointer to the associated depot
    double angleCenter;  // Angle of the barycenter of the route

    /**
     * Tests if this route is feasible.
     */
    [[nodiscard]] bool isFeasible() const
    {
        return !hasExcessCapacity() && !hasTimeWarp();
    }

    /**
     * Determines whether this route is load-feasible.
     */
    [[nodiscard]] bool hasExcessCapacity() const
    {
        return load() > params->vehicleCapacity;
    }

    /**
     * Determines whether this route is time-feasible.
     */
    [[nodiscard]] bool hasTimeWarp() const { return timeWarp() > 0; }

    /**
     * Returns total load on this route.
     */
    [[nodiscard]] int load() const { return nodes.back()->cumulatedLoad; }

    /**
     * Returns total time warp on this route.
     */
    [[nodiscard]] int timeWarp() const
    {
        auto const &tw = nodes.back()->twBefore;
        return tw.totalTimeWarp();
    }

    /**
     * Tests if this route overlaps the other route, that is, whether their
     * circle sectors overlap with a given tolerance.
     */
    [[nodiscard]] bool overlapsWith(Route const &other) const
    {
        return CircleSector::overlap(
            sector, other.sector, params->config.circleSectorOverlapTolerance);
    }

    [[nodiscard]] bool empty() const { return size() == 0; }

    [[nodiscard]] size_t size() const
    {
        return nodes.size() - 1;  // exclude end depot
    }

    /**
     * Calculates time window data for segment [start, end].
     */
    [[nodiscard]] TimeWindowSegment twBetween(size_t start, size_t end) const;

    /**
     * Calculates the distance for segment [start, end].
     */
    [[nodiscard]] inline int distBetween(size_t start, size_t end) const;

    /**
     * Calculates the load for segment [start, end].
     */
    [[nodiscard]] inline int loadBetween(size_t start, size_t end) const;

    /**
     * Updates this route. To be called after swapping nodes/changing the
     * solution.
     */
    void update();
};

int Route::distBetween(size_t start, size_t end) const
{
    assert(start <= end && end <= nodes.size());

    auto const startDist = start == 0 ? 0 : nodes[start - 1]->cumulatedDistance;
    auto const endDist = nodes[end - 1]->cumulatedDistance;

    assert(startDist <= endDist);

    return endDist - startDist;
}

int Route::loadBetween(size_t start, size_t end) const
{
    assert(start <= end && end <= nodes.size());

    auto const *startNode = start == 0 ? depot : nodes[start - 1];
    auto const atStart = params->clients[startNode->client].demand;
    auto const startLoad = startNode->cumulatedLoad;
    auto const endLoad = nodes[end - 1]->cumulatedLoad;

    assert(startLoad <= endLoad);

    return endLoad - startLoad + atStart;
}

// Outputs a route into a given ostream in CVRPLib format
std::ostream &operator<<(std::ostream &out, Route const &route);

#endif  // HGS_VRPTW_ROUTE_H
