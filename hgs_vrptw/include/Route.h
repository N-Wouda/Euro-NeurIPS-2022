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
            : from(from), to(to), tw(from->route->twBetween(from, to))
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

    int idx;                    // Route index
    Node *depot;                // Pointer to the associated depot
    int load;                   // Total load on the route
    TimeWindowSegment twData;   // Time window data of the route
    int penalty;                // Current load and time window penalties
    double angleCenter;         // Angle of the barycenter of the route
    int whenLastModified;       // "When" this route has been last modified
    int whenLastTestedLargeNb;  // "When" the large neighborhood moves of
                                // this route have last been tested

    /**
     * Tests if this route is feasible.
     */
    [[nodiscard]] inline bool isFeasible() const
    {
        return !hasExcessCapacity() && !hasTimeWarp();
    }

    /**
     * Determines whether this route is load-feasible.
     */
    [[nodiscard]] inline bool hasExcessCapacity() const
    {
        return load > params->vehicleCapacity;
    }

    /**
     * Determines whether this route is time-feasible.
     */
    [[nodiscard]] inline bool hasTimeWarp() const
    {
        return twData.totalTimeWarp() > 0;
    }

    [[nodiscard]] inline bool overlapsWith(Route const &other) const
    {
        return CircleSector::overlap(
            sector, other.sector, params->config.circleSectorOverlapTolerance);
    }

    [[nodiscard]] inline bool empty() const { return nbCustomers == 0; }

    /**
     * Calculates time window data for segment [start, end] in this route.
     */
    TimeWindowSegment twBetween(Node const *start, Node const *end) const;

    /**
     * Updates this route. To be called after swapping nodes/changing the
     * solution.
     */
    void update(int nbMoves, Penalties const &penalties);
};

#endif  // HGS_VRPTW_ROUTE_H
