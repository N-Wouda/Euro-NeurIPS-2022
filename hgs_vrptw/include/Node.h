#ifndef NODE_H
#define NODE_H

#include "TimeWindowSegment.h"

class Route;

class Node
{
public:  // TODO make fields private
    Params const *params;

    int client;            // Client represented with this node
    size_t position;       // Position in the route
    Node *next;            // Next node in the route order
    Node *prev;            // Previous node in the route order
    Route *route;          // Pointer towards the associated route
    int cumulatedLoad;     // Cumulative load from depot to client (inclusive)

    // Distance difference if the segment (0...client) is reversed. This is
    // useful for 2-OPT moves in asymmetric problems.
    int cumulatedReversalDistance;

    TimeWindowSegment tw;        // TWS for individual node (client)
    TimeWindowSegment twBefore;  // TWS for (0...client) including self
    TimeWindowSegment twAfter;   // TWS for (client...0) including self

    [[nodiscard]] inline bool isDepot() const { return client == 0; }

    /**
     * Inserts this node after the other and updates the solution.
     */
    void insertAfter(Node *other);

    /**
     * Swaps this node with the other and updates the solution.
     */
    void swapWith(Node *other);
};

/**
 * Convenience method accessing the node directly before the argument.
 */
inline Node *p(Node *node) { return node->prev; }

/**
 * Convenience method accessing the node two positions before the argument.
 */
inline Node *pp(Node *node) { return node->prev->prev; }

/**
 * Convenience method accessing the node directly after the argument.
 */
inline Node *n(Node *node) { return node->next; }

/**
 * Convenience method accessing the node two positions after the argument.
 */
inline Node *nn(Node *node) { return node->next->next; }

#endif  // NODE_H
