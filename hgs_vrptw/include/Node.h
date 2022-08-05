#ifndef NODE_H
#define NODE_H

#include "TimeWindowSegment.h"
#include <iostream>

struct Route;

struct Node
{
    Params const *params;

    bool isDepot;          // Tells whether this node represents a depot or not
    int client;            // Node index
    int position;          // Position in the route
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
    int deltaRemoval;      // Difference of cost in the current route if the
                           // node is removed (used in SWAP*)
    int deltaRemovalTW;    // Difference of cost in the current route if the
                           // node is removed, including TimeWarp (used in
                           // SWAP*)
    TimeWindowSegment tw;  // TimeWindowSegment for individual node (client)

    // TimeWindowSegment for subsequence (0...client) including self
    TimeWindowSegment twBefore;

    // TimeWindowSegment for subsequence (client...0) including self
    TimeWindowSegment twAfter;

    bool isSeed;  // Tells whether a nextSeed is available (faster tw
                  // calculations)
    TimeWindowSegment toNextSeedTwD;  // TimeWindowSegment for subsequence
                                      // (client...client+4) excluding self,
                                      // including client + 4
    Node *nextSeed;  // next seeded node if available (nullptr otherwise)

    // Calculates time window data for segment [self, other] in same route
    TimeWindowSegment mergeSegmentTwData(Node const *other) const;
};

#endif  // NODE_H
