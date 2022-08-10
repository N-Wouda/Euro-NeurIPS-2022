#ifndef HGS_VRPTW_ROUTE_H
#define HGS_VRPTW_ROUTE_H

#include "CircleSector.h"
#include "Node.h"
#include "Penalties.h"
#include "TimeWindowSegment.h"

#include <array>
#include <bit>

class Route
{
    // These fields are used for some internal jump point calculations. Can be
    // updated with new jump point offsets.
    constexpr static std::array<size_t, 2> jumpPts = {4, 8};
    constexpr static size_t jumpOffset = std::bit_width(4UL);

    struct JumpNode
    {
        Node const *from = nullptr;
        Node const *to = nullptr;
        TimeWindowSegment tw;

        JumpNode() = default;

        JumpNode(Node const *from, Node const *to)
            : from(from), to(to), tw(Route::twBetween(from, to))
        {
        }

        JumpNode(Node const *from, Node const *to, TimeWindowSegment tw)
            : from(from), to(to), tw(tw)
        {
        }
    };

    // Vector of jump distance by nodes. Each element is a JumpNode, which lets
    // us jump to the next node in the route a given jump distance away. Very
    // useful for speeding up time window calculations along the route.
    std::vector<std::vector<JumpNode>> jumps;
    size_t nbCustomers;         // Number of customers in the route
    CircleSector sector;        // Circle sector of the route's clients
    std::vector<Node *> nodes;  // List of nodes (in order) in this solution.

    // Sets jump points, pointing to the current node from earlier route nodes.
    void installJumpPoints(Node const *node);

public:  // TODO make fields private
    Params const *params;

    int idx;               // Route index
    Node *depot;           // Pointer to the associated depot
    int load;              // Total load on the route
    TimeWindowSegment tw;  // Time window data of the route
    double angleCenter;    // Angle of the barycenter of the route

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
        return load > params->vehicleCapacity;
    }

    /**
     * Determines whether this route is time-feasible.
     */
    [[nodiscard]] bool hasTimeWarp() const { return tw.totalTimeWarp() > 0; }

    [[nodiscard]] bool overlapsWith(Route const &other) const
    {
        return CircleSector::overlap(
            sector, other.sector, params->config.circleSectorOverlapTolerance);
    }

    [[nodiscard]] bool empty() const { return nbCustomers == 0; }

    /**
     * Calculates time window data for segment [start, end] in the same route.
     */
    static TimeWindowSegment twBetween(Node const *start, Node const *end);

    /**
     * Calculates the distance for segment [start, end] in the same route.
     */
    static int distBetween(Node const *start, Node const *end);

    /**
     * Calculates the load for segment [start, end] in the same route.
     */
    static int loadBetween(Node const *start, Node const *end);

    /**
     * Updates this route. To be called after swapping nodes/changing the
     * solution.
     */
    void update();
};

#endif  // HGS_VRPTW_ROUTE_H
