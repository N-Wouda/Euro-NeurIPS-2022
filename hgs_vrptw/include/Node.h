#ifndef NODE_H
#define NODE_H

#include "TimeWindowSegment.h"

class Route;

class Node
{
public:  // TODO make fields private
    Params const *params;

    bool isDepot;          // Tells whether this node represents a depot or not
    int client;            // Node index
    size_t position;       // Position in the route
    int whenLastTestedRI;  // "When" the RI moves for this node have been
                           // last tested
    Node *next;            // Next node in the route order
    Node *prev;            // Previous node in the route order
    Route *route;          // Pointer towards the associated route
    int cumulatedLoad;     // Cumulated load on this route until the client
                           // (including itself)
    int cumulatedReversalDistance;  // Difference of cost if the segment of
                                    // route (0...client) is reversed
                                    // (useful for 2-opt moves with
                                    // asymmetric problems)

    TimeWindowSegment tw;        // TWS for individual node (client)
    TimeWindowSegment twBefore;  // TWS for (0...client) including self
    TimeWindowSegment twAfter;   // TWS for (client...0) including self

    /**
     * Inserts this node after the other and updates the solution.
     */
    void insertAfter(Node *other);

    /**
     * Swaps this node with the other and updates the solution.
     */
    void swapWith(Node *other);
};

#endif  // NODE_H
