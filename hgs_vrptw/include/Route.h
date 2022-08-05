#ifndef HGS_VRPTW_ROUTE_H
#define HGS_VRPTW_ROUTE_H

#include "CircleSector.h"
#include "Node.h"
#include "Penalties.h"
#include "TimeWindowSegment.h"

struct Route
{
    Params const *params;

    int idx;                      // Route index
    int nbCustomers;              // Number of customers in the route
    Node *depot;                  // Pointer to the associated depot
    int load;                     // Total load on the route
    TimeWindowSegment twData;     // Time window data of the route
    int penalty;                  // Current load and time window penalties
    double polarAngleBarycenter;  // Angle of the barycenter of the route
    CircleSector sector;          // Circle sector of the route's clients
    int whenLastModified;         // "When" this route has been last modified
    int whenLastTestedLargeNb;    // "When" the large neighborhood moves of
                                  // this route have last been tested

    /**
     * Updates this route. To be called after swapping nodes/changing the
     * solution.
     */
    void update(int nbMoves, Penalties const &penalties);
};

#endif  // HGS_VRPTW_ROUTE_H
