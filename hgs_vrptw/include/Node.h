#ifndef NODE_H
#define NODE_H

#include "Route.h"
#include "TimeWindowSegment.h"

#include <cassert>

struct Node
{
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
    TimeWindowSegment twBefore;  // TimeWindowSegment for subsequence
                                 // (0...client) including self
    TimeWindowSegment twAfter;   // TimeWindowSegment for subsequence
                                 // (client...0) including self
    bool isSeed;  // Tells whether a nextSeed is available (faster tw
                  // calculations)
    TimeWindowSegment toNextSeedTwD;  // TimeWindowSegment for subsequence
                                      // (client...client+4) excluding self,
                                      // including client + 4
    Node *nextSeed;  // next seeded node if available (nullptr otherwise)

    // Calculates time window data for segment [self, other] in same route
    TimeWindowSegment mergeSegmentTwData(Node *other)
    {
        assert(route == other->route);
        assert(position <= other->position);

        if (isDepot)
            return other->twBefore;

        if (other->isDepot)
            return twAfter;

        Node *node = this;
        TimeWindowSegment data = tw;

        while (node != other)
        {
            if (node->isSeed && node->position + 4 <= other->position)
            {
                data = TimeWindowSegment::merge(data, node->toNextSeedTwD);
                node = node->nextSeed;
            }
            else
            {
                node = node->next;
                data = TimeWindowSegment::merge(data, node->tw);
            }
        }

        return data;
    }
};

#endif  // NODE_H
