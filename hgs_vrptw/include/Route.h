#ifndef HGS_VRPTW_ROUTE_H
#define HGS_VRPTW_ROUTE_H

#include "CircleSector.h"
#include "TimeWindowSegment.h"

struct Node;

struct Route
{
    int idx;                        // Route index
    int nbCustomers;                // Number of customers visited in the route
    int whenLastModified;           // "When" this route has been last modified
    int whenLastTestedLargeNb;      // "When" the large neighborhood moves for
                                    // this route have been last tested
    bool isDeltaRemovalTWOutdated;  // Flag to indicate deltaRemovalTW data
                                    // of nodes is outdated
    Node *depot;                    // Pointer to the associated depot
    int load;                       // Total load on the route
    TimeWindowSegment twData;       // Time window data of the route
    int penalty;  // Current sum of load, duration and time window penalties
    double polarAngleBarycenter;  // Polar angle of the barycenter of the route
    CircleSector sector;  // Circle sector associated to the set of clients
};

#endif  // HGS_VRPTW_ROUTE_H
