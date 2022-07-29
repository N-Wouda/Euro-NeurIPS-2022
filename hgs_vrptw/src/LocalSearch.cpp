#include "LocalSearch.h"

#include "CircleSector.h"
#include "Individual.h"
#include "Params.h"

#include <cmath>
#include <numeric>
#include <set>
#include <vector>

void LocalSearch::operator()(Individual &indiv,
                             int excessCapacityPenalty,
                             int timeWarpPenalty)
{
    penaltyCapacityLS = excessCapacityPenalty;
    penaltyTimeWarpLS = timeWarpPenalty;

    // Shuffling the node order beforehand adds diversity to the search
    std::shuffle(orderNodes.begin(), orderNodes.end(), rng);
    std::shuffle(orderRoutes.begin(), orderRoutes.end(), rng);

    loadIndividual(indiv);       // load individual...
    search();                    // ...perform local search...
    indiv = exportIndividual();  // ...export result back into the individual
}

void LocalSearch::search()
{
    bool const shouldIntensify
        = rng.randint(100) < (unsigned)params.config.intensificationProbability;

    searchCompleted = false;
    for (int step = 0; !searchCompleted; ++step)
    {
        if (step > 1)                // At least two loops as some moves with
            searchCompleted = true;  // empty routes are not done in the first

        /* ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY RESTRICTION */
        for (int posU = 0; posU < params.nbClients; posU++)
        {
            auto &nodeU = clients[orderNodes[posU]];

            int lastTestRINodeU = nodeU.whenLastTestedRI;
            nodeU.whenLastTestedRI = nbMoves;

            for (auto const &vIdx : params.getNeighboursOf(nodeU.cour))
            {
                auto &nodeV = clients[vIdx];

                if (step == 0
                    || std::max(nodeU.route->whenLastModified,
                                nodeV.route->whenLastModified)
                           > lastTestRINodeU)  // only evaluate moves involving
                                               // routes that have been
                                               // modified since last move
                                               // evaluations for nodeU
                {
                    // Randomizing the order of the neighborhoods within this
                    // loop does not matter much as we are already randomizing
                    // the order of the node pairs (and it's not very common to
                    // find improving moves of different types for the same node
                    // pair)
                    u = {params, &nodeU};
                    v = {params, &nodeV};

                    if (MoveSingleClient())
                        continue;  // RELOCATE
                    if (MoveTwoClients())
                        continue;  // RELOCATE
                    if (MoveTwoClientsReversed())
                        continue;  // RELOCATE
                    if (SwapTwoSingleClients())
                        continue;  // SWAP
                    if (SwapTwoClientsForOne())
                        continue;  // SWAP
                    if (SwapTwoClientPairs())
                        continue;  // SWAP
                    if (TwoOptBetweenTrips())
                        continue;  // 2-OPT*
                    if (TwoOptWithinTrip())
                        continue;  // 2-OPT

                    // Trying moves that insert nodeU directly after the depot
                    if (nodeV.prev->isDepot)
                    {
                        v = {params, nodeV.prev};

                        if (MoveSingleClient())
                            continue;  // RELOCATE
                        if (MoveTwoClients())
                            continue;  // RELOCATE
                        if (MoveTwoClientsReversed())
                            continue;  // RELOCATE
                        if (TwoOptBetweenTrips())
                            continue;  // 2-OPT*
                    }
                }
            }

            /* MOVES INVOLVING AN EMPTY ROUTE -- NOT TESTED IN THE FIRST LOOP TO
             * AVOID INCREASING TOO MUCH THE FLEET SIZE */
            if (step > 0 && !emptyRoutes.empty())
            {
                v = {params, routes[*emptyRoutes.begin()].depot};

                if (MoveSingleClient())
                    continue;  // RELOCATE
                if (MoveTwoClients())
                    continue;  // RELOCATE
                if (MoveTwoClientsReversed())
                    continue;  // RELOCATE
                if (TwoOptBetweenTrips())
                    continue;  // 2-OPT*
            }
        }

        /* (SWAP*) MOVES LIMITED TO ROUTE PAIRS WHOSE CIRCLE SECTORS OVERLAP */
        if (searchCompleted && shouldIntensify)
        {
            u = {params, u.node};
            v = {params, v.node};

            for (int rU = 0; rU < params.nbVehicles; rU++)
            {
                u.route = &routes[orderRoutes[rU]];
                if (u.route->nbCustomers == 0)
                    continue;

                int lastTestLargeNbRouteU = u.route->whenLastTestedLargeNb;
                u.route->whenLastTestedLargeNb = nbMoves;
                for (int rV = 0; rV < params.nbVehicles; rV++)
                {
                    v.route = &routes[orderRoutes[rV]];
                    if (v.route->nbCustomers == 0
                        || u.route->cour >= v.route->cour)
                        continue;

                    if (step > 0
                        && std::max(u.route->whenLastModified,
                                    v.route->whenLastModified)
                               <= lastTestLargeNbRouteU)
                        continue;

                    if (!CircleSector::overlap(
                            u.route->sector,
                            v.route->sector,
                            params.config.circleSectorOverlapTolerance))
                        continue;

                    if (!RelocateStar())
                        if (params.config.skipSwapStarDist || !swapStar(false))
                            if (params.config.useSwapStarTW)
                                swapStar(true);
                }
            }
        }
    }
}

bool LocalSearch::MoveSingleClient()
{
    // If U already comes directly after V, this move has no effect
    if (u.nodeIndex == v.nextIndex)
        return false;

    int costSuppU = params.dist(u.prevIndex, u.nextIndex)
                       - params.dist(u.prevIndex, u.nodeIndex)
                       - params.dist(u.nodeIndex, u.nextIndex);
    int costSuppV = params.dist(v.nodeIndex, u.nodeIndex)
                       + params.dist(u.nodeIndex, v.nextIndex)
                       - params.dist(v.nodeIndex, v.nextIndex);

    if (u.route != v.route)
    {
        if (!u.hasExcessLoad && !u.hasTimeWarp
            && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = mergeTwDataRecursive(u.node->prev->prefixTwData,
                                                 u.nextNode->postfixTwData);
        auto routeVTwData = mergeTwDataRecursive(
            v.node->prefixTwData, u.node->twData, v.nextNode->postfixTwData);

        costSuppU += penaltyExcessLoad(u.route->load - u.demand)
                     + penaltyTimeWindows(routeUTwData) - u.route->penalty;

        costSuppV += penaltyExcessLoad(v.route->load + u.demand)
                     + penaltyTimeWindows(routeVTwData) - v.route->penalty;
    }
    else
    {
        if (!u.hasTimeWarp && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Move within the same route
        if (u.node->position < v.node->position)
        {
            // Edge case V directly after U, so X == V, this works
            // start - ... - UPrev - X - ... - V - U - Y - ... - end
            auto const routeUTwData = mergeTwDataRecursive(
                u.node->prev->prefixTwData,
                getRouteSegmentTwData(u.nextNode, v.node),
                u.node->twData,
                v.nextNode->postfixTwData);

            costSuppU += penaltyTimeWindows(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - U - Y - ... - UPrev - X - ... - end
            auto const routeUTwData = mergeTwDataRecursive(
                v.node->prefixTwData,
                u.node->twData,
                getRouteSegmentTwData(v.nextNode, u.node->prev),
                u.nextNode->postfixTwData);

            costSuppU += penaltyTimeWindows(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penaltyExcessLoad(u.route->load) - u.route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    insertNode(u.node, v.node);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(u.route);
    if (u.route != v.route)
        updateRouteData(v.route);

    return true;
}

bool LocalSearch::MoveTwoClients()
{
    if (u.node == v.nextNode || v.node == u.nextNode || u.nextNode->isDepot)
        return false;

    int costSuppU = params.dist(u.prevIndex, u.nextNextIndex)
                       - params.dist(u.prevIndex, u.nodeIndex)
                       - params.dist(u.nextIndex, u.nextNextIndex);
    int costSuppV = params.dist(v.nodeIndex, u.nodeIndex)
                       + params.dist(u.nextIndex, v.nextIndex)
                       - params.dist(v.nodeIndex, v.nextIndex);

    if (u.route != v.route)
    {
        if (!u.hasExcessLoad && !u.hasTimeWarp
            && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = mergeTwDataRecursive(
            u.node->prev->prefixTwData, u.nextNode->next->postfixTwData);
        auto routeVTwData
            = mergeTwDataRecursive(v.node->prefixTwData,
                                   getEdgeTwData(u.node, u.nextNode),
                                   v.nextNode->postfixTwData);

        costSuppU += penaltyExcessLoad(u.route->load - u.demand - u.nextDemand)
                     + penaltyTimeWindows(routeUTwData) - u.route->penalty;

        costSuppV += penaltyExcessLoad(v.route->load + u.demand + u.nextDemand)
                     + penaltyTimeWindows(routeVTwData) - v.route->penalty;
    }
    else
    {
        if (!u.hasTimeWarp && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Move within the same route
        if (u.node->position < v.node->position)
        {
            // Edge case V directly after U, so X == V is excluded, V directly
            // after X so XNext == V works start - ... - UPrev - XNext - ... - V
            // - U - X - Y - ... - end
            auto const routeUTwData = mergeTwDataRecursive(
                u.node->prev->prefixTwData,
                getRouteSegmentTwData(u.nextNode->next, v.node),
                getEdgeTwData(u.node, u.nextNode),
                v.nextNode->postfixTwData);

            costSuppU += penaltyTimeWindows(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - U - X - Y - ... - UPrev - XNext - ...
            // - end
            auto const routeUTwData = mergeTwDataRecursive(
                v.node->prefixTwData,
                getEdgeTwData(u.node, u.nextNode),
                getRouteSegmentTwData(v.nextNode, u.node->prev),
                u.nextNode->next->postfixTwData);

            costSuppU += penaltyTimeWindows(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penaltyExcessLoad(u.route->load) - u.route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    insertNode(u.node, v.node);
    insertNode(u.nextNode, u.node);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(u.route);
    if (u.route != v.route)
        updateRouteData(v.route);

    return true;
}

bool LocalSearch::MoveTwoClientsReversed()
{
    if (u.node == v.nextNode || u.nextNode == v.node || u.nextNode->isDepot)
        return false;

    int costSuppU = params.dist(u.prevIndex, u.nextNextIndex)
                       - params.dist(u.prevIndex, u.nodeIndex)
                       - params.dist(u.nodeIndex, u.nextIndex)
                       - params.dist(u.nextIndex, u.nextNextIndex);
    int costSuppV = params.dist(v.nodeIndex, u.nextIndex)
                       + params.dist(u.nextIndex, u.nodeIndex)
                       + params.dist(u.nodeIndex, v.nextIndex)
                       - params.dist(v.nodeIndex, v.nextIndex);

    if (u.route != v.route)
    {
        if (!u.hasExcessLoad && !u.hasTimeWarp
            && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = mergeTwDataRecursive(
            u.node->prev->prefixTwData, u.nextNode->next->postfixTwData);
        auto routeVTwData
            = mergeTwDataRecursive(v.node->prefixTwData,
                                   getEdgeTwData(u.nextNode, u.node),
                                   v.nextNode->postfixTwData);

        costSuppU += penaltyExcessLoad(u.route->load - u.demand - u.nextDemand)
                     + penaltyTimeWindows(routeUTwData) - u.route->penalty;

        costSuppV += penaltyExcessLoad(v.route->load + u.demand + u.nextDemand)
                     + penaltyTimeWindows(routeVTwData) - v.route->penalty;
    }
    else
    {
        if (!u.hasTimeWarp && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Move within the same route
        if (u.node->position < v.node->position)
        {
            // Edge case V directly after U, so X == V is excluded, V directly
            // after X so XNext == V works start - ... - UPrev - XNext - ... - V
            // - X - U - Y - ... - end
            auto const routeUTwData = mergeTwDataRecursive(
                u.node->prev->prefixTwData,
                getRouteSegmentTwData(u.nextNode->next, v.node),
                getEdgeTwData(u.nextNode, u.node),
                v.nextNode->postfixTwData);

            costSuppU += penaltyTimeWindows(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - X - U - Y - ... - UPrev - XNext - ...
            // - end
            auto const routeUTwData = mergeTwDataRecursive(
                v.node->prefixTwData,
                getEdgeTwData(u.nextNode, u.node),
                getRouteSegmentTwData(v.nextNode, u.node->prev),
                u.nextNode->next->postfixTwData);

            costSuppU += penaltyTimeWindows(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penaltyExcessLoad(u.route->load) - u.route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    insertNode(u.nextNode, v.node);
    insertNode(u.node, u.nextNode);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(u.route);
    if (u.route != v.route)
        updateRouteData(v.route);

    return true;
}

bool LocalSearch::SwapTwoSingleClients()
{
    if (u.nodeIndex >= v.nodeIndex)
        return false;

    if (u.nodeIndex == v.prevIndex || u.nodeIndex == v.nextIndex)
        return false;

    int costSuppU = params.dist(u.prevIndex, v.nodeIndex)
                       + params.dist(v.nodeIndex, u.nextIndex)
                       - params.dist(u.prevIndex, u.nodeIndex)
                       - params.dist(u.nodeIndex, u.nextIndex);
    int costSuppV = params.dist(v.prevIndex, u.nodeIndex)
                       + params.dist(u.nodeIndex, v.nextIndex)
                       - params.dist(v.prevIndex, v.nodeIndex)
                       - params.dist(v.nodeIndex, v.nextIndex);

    if (u.route != v.route)
    {
        if (!u.hasExcessLoad && !u.hasTimeWarp && !v.hasExcessLoad
            && !v.hasTimeWarp && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = mergeTwDataRecursive(u.node->prev->prefixTwData,
                                                 v.node->twData,
                                                 u.nextNode->postfixTwData);
        auto routeVTwData = mergeTwDataRecursive(v.node->prev->prefixTwData,
                                                 u.node->twData,
                                                 v.nextNode->postfixTwData);

        costSuppU += penaltyExcessLoad(u.route->load + v.demand - u.demand)
                     + penaltyTimeWindows(routeUTwData) - u.route->penalty;

        costSuppV += penaltyExcessLoad(v.route->load + u.demand - v.demand)
                     + penaltyTimeWindows(routeVTwData) - v.route->penalty;
    }
    else
    {
        if (!u.hasTimeWarp && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Swap within the same route
        if (u.node->position < v.node->position)
        {
            // Edge case V directly after U, so X == V is excluded, V directly
            // after X so XNext == V works start - ... - UPrev - V - X - ... -
            // VPrev - U - Y - ... - end
            auto const routeUTwData = mergeTwDataRecursive(
                u.node->prev->prefixTwData,
                v.node->twData,
                getRouteSegmentTwData(u.nextNode, v.node->prev),
                u.node->twData,
                v.nextNode->postfixTwData);

            costSuppU += penaltyTimeWindows(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - VPrev - U - Y - ... - UPrev - V - X - ...
            // - end
            auto const routeUTwData = mergeTwDataRecursive(
                v.node->prev->prefixTwData,
                u.node->twData,
                getRouteSegmentTwData(v.nextNode, u.node->prev),
                v.node->twData,
                u.nextNode->postfixTwData);

            costSuppU += penaltyTimeWindows(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penaltyExcessLoad(u.route->load) - u.route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    swapNode(u.node, v.node);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(u.route);
    if (u.route != v.route)
        updateRouteData(v.route);

    return true;
}

bool LocalSearch::SwapTwoClientsForOne()
{
    if (u.node == v.node->prev || u.nextNode == v.node->prev
        || u.node == v.nextNode || u.nextNode->isDepot)
        return false;

    int costSuppU = params.dist(u.prevIndex, v.nodeIndex)
                       + params.dist(v.nodeIndex, u.nextNextIndex)
                       - params.dist(u.prevIndex, u.nodeIndex)
                       - params.dist(u.nextIndex, u.nextNextIndex);
    int costSuppV = params.dist(v.prevIndex, u.nodeIndex)
                       + params.dist(u.nextIndex, v.nextIndex)
                       - params.dist(v.prevIndex, v.nodeIndex)
                       - params.dist(v.nodeIndex, v.nextIndex);

    if (u.route != v.route)
    {
        if (!u.hasExcessLoad && !u.hasTimeWarp && !v.hasExcessLoad
            && !v.hasTimeWarp && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData
            = mergeTwDataRecursive(u.node->prev->prefixTwData,
                                   v.node->twData,
                                   u.nextNode->next->postfixTwData);
        auto routeVTwData
            = mergeTwDataRecursive(v.node->prev->prefixTwData,
                                   getEdgeTwData(u.node, u.nextNode),
                                   v.nextNode->postfixTwData);

        costSuppU += penaltyExcessLoad(u.route->load + v.demand - u.demand
                                       - u.nextDemand)
                     + penaltyTimeWindows(routeUTwData) - u.route->penalty;

        costSuppV += penaltyExcessLoad(v.route->load + u.demand + u.nextDemand
                                       - v.demand)
                     + penaltyTimeWindows(routeVTwData) - v.route->penalty;
    }
    else
    {
        if (!u.hasTimeWarp && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Swap within the same route
        if (u.node->position < v.node->position)
        {
            // start - ... - UPrev - V - XNext - ... - VPrev - U - X - Y - ... -
            // end
            auto const routeUTwData = mergeTwDataRecursive(
                u.node->prev->prefixTwData,
                v.node->twData,
                getRouteSegmentTwData(u.nextNode->next, v.node->prev),
                getEdgeTwData(u.node, u.nextNode),
                v.nextNode->postfixTwData);

            costSuppU += penaltyTimeWindows(routeUTwData);
        }
        else
        {
            // start - ... - VPrev - U - X - Y - ... - UPrev - V - XNext - ... -
            // end
            auto const routeUTwData = mergeTwDataRecursive(
                v.node->prev->prefixTwData,
                getEdgeTwData(u.node, u.nextNode),
                getRouteSegmentTwData(v.nextNode, u.node->prev),
                v.node->twData,
                u.nextNode->next->postfixTwData);

            costSuppU += penaltyTimeWindows(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penaltyExcessLoad(u.route->load) - u.route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    // Note: next two lines are a bit inefficient but we only update
    // occasionally and updateRouteData is much more costly anyway, efficient
    // checks are more important
    swapNode(u.node, v.node);
    insertNode(u.nextNode, u.node);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(u.route);
    if (u.route != v.route)
        updateRouteData(v.route);

    return true;
}

bool LocalSearch::SwapTwoClientPairs()
{
    if (u.nodeIndex >= v.nodeIndex)
        return false;

    if (u.nextNode->isDepot || v.nextNode->isDepot || v.nextNode == u.node->prev
        || u.node == v.nextNode || u.nextNode == v.node
        || v.node == u.nextNode->next)
        return false;

    int costSuppU = params.dist(u.prevIndex, v.nodeIndex)
                       + params.dist(v.nextIndex, u.nextNextIndex)
                       - params.dist(u.prevIndex, u.nodeIndex)
                       - params.dist(u.nextIndex, u.nextNextIndex);
    int costSuppV = params.dist(v.prevIndex, u.nodeIndex)
                       + params.dist(u.nextIndex, v.nextNextIndex)
                       - params.dist(v.prevIndex, v.nodeIndex)
                       - params.dist(v.nextIndex, v.nextNextIndex);

    if (u.route != v.route)
    {
        if (!u.hasExcessLoad && !u.hasTimeWarp && !v.hasExcessLoad
            && !v.hasTimeWarp && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData
            = mergeTwDataRecursive(u.node->prev->prefixTwData,
                                   getEdgeTwData(v.node, v.nextNode),
                                   u.nextNode->next->postfixTwData);
        auto routeVTwData
            = mergeTwDataRecursive(v.node->prev->prefixTwData,
                                   getEdgeTwData(u.node, u.nextNode),
                                   v.nextNode->next->postfixTwData);

        costSuppU += penaltyExcessLoad(u.route->load + v.demand + v.nextDemand
                                       - u.demand - u.nextDemand)
                     + penaltyTimeWindows(routeUTwData) - u.route->penalty;

        costSuppV += penaltyExcessLoad(v.route->load + u.demand + u.nextDemand
                                       - v.demand - v.nextDemand)
                     + penaltyTimeWindows(routeVTwData) - v.route->penalty;
    }
    else
    {
        if (!u.hasTimeWarp && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Swap within the same route
        if (u.node->position < v.node->position)
        {
            // start - ... - UPrev - V - Y - XNext - ... - VPrev - U - X - YNext
            // - ... - end
            auto const routeUTwData = mergeTwDataRecursive(
                u.node->prev->prefixTwData,
                getEdgeTwData(v.node, v.nextNode),
                getRouteSegmentTwData(u.nextNode->next, v.node->prev),
                getEdgeTwData(u.node, u.nextNode),
                v.nextNode->next->postfixTwData);

            costSuppU += penaltyTimeWindows(routeUTwData);
        }
        else
        {
            // start - ... - VPrev - U - X - YNext - ... - UPrev - V - Y - XNext
            // - ... - end
            auto const routeUTwData = mergeTwDataRecursive(
                v.node->prev->prefixTwData,
                getEdgeTwData(u.node, u.nextNode),
                getRouteSegmentTwData(v.nextNode->next, u.node->prev),
                getEdgeTwData(v.node, v.nextNode),
                u.nextNode->next->postfixTwData);

            costSuppU += penaltyTimeWindows(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penaltyExcessLoad(u.route->load) - u.route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    swapNode(u.node, v.node);
    swapNode(u.nextNode, v.nextNode);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(u.route);
    if (u.route != v.route)
        updateRouteData(v.route);

    return true;
}

bool LocalSearch::TwoOptWithinTrip()
{
    if (u.route != v.route)
        return false;

    if (u.node->position >= v.node->position - 1)
        return false;

    int cost = params.dist(u.nodeIndex, v.nodeIndex)
                  + params.dist(u.nextIndex, v.nextIndex)
                  - params.dist(u.nodeIndex, u.nextIndex)
                  - params.dist(v.nodeIndex, v.nextIndex)
                  + v.node->cumulatedReversalDistance
                  - u.nextNode->cumulatedReversalDistance;

    if (!u.hasTimeWarp && cost >= 0)
    {
        return false;
    }

    TimeWindowData routeTwData = u.node->prefixTwData;
    Node *itRoute = v.node;
    while (itRoute != u.node)
    {
        routeTwData = mergeTwDataRecursive(routeTwData, itRoute->twData);
        itRoute = itRoute->prev;
    }
    routeTwData = mergeTwDataRecursive(routeTwData, v.nextNode->postfixTwData);

    // Compute new total penalty
    cost += penaltyExcessLoad(u.route->load) + penaltyTimeWindows(routeTwData)
            - u.route->penalty;

    if (cost >= 0)
    {
        return false;
    }

    itRoute = v.node;
    Node *insertionPoint = u.node;
    while (itRoute != u.nextNode)  // No need to move x, we pivot around it
    {
        Node *current = itRoute;
        itRoute = itRoute->prev;
        insertNode(current, insertionPoint);
        insertionPoint = current;
    }

    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(u.route);

    return true;
}

bool LocalSearch::TwoOptBetweenTrips()
{
    if (u.route->cour >= v.route->cour)
        return false;

    int costSuppU = params.dist(u.nodeIndex, v.nextIndex)
                       - params.dist(u.nodeIndex, u.nextIndex);
    int costSuppV = params.dist(v.nodeIndex, u.nextIndex)
                       - params.dist(v.nodeIndex, v.nextIndex);

    if (!u.hasExcessLoad && !u.hasTimeWarp && !v.hasExcessLoad && !v.hasTimeWarp
        && costSuppU + costSuppV >= 0)
    {
        return false;
    }

    auto routeUTwData
        = mergeTwDataRecursive(u.node->prefixTwData, v.nextNode->postfixTwData);
    auto routeVTwData
        = mergeTwDataRecursive(v.node->prefixTwData, u.nextNode->postfixTwData);

    costSuppU += penaltyExcessLoad(u.node->cumulatedLoad + v.route->load
                                   - v.node->cumulatedLoad)
                 + penaltyTimeWindows(routeUTwData) - u.route->penalty;

    costSuppV += penaltyExcessLoad(v.node->cumulatedLoad + u.route->load
                                   - u.node->cumulatedLoad)
                 + penaltyTimeWindows(routeVTwData) - v.route->penalty;

    if (costSuppU + costSuppV >= 0)
        return false;

    Node *itRouteV = v.nextNode;
    Node *insertLocation = u.node;
    while (!itRouteV->isDepot)
    {
        Node *current = itRouteV;
        itRouteV = itRouteV->next;
        insertNode(current, insertLocation);
        insertLocation = current;
    }

    Node *itRouteU = u.nextNode;
    insertLocation = v.node;
    while (!itRouteU->isDepot)
    {
        Node *current = itRouteU;
        itRouteU = itRouteU->next;
        insertNode(current, insertLocation);
        insertLocation = current;
    }

    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(u.route);
    updateRouteData(v.route);

    return true;
}

bool LocalSearch::swapStar(const bool withTW)
{
    SwapStarElement bestSwap;

    if (!bestInsertInitializedForRoute[u.route->cour])
    {
        bestInsertInitializedForRoute[u.route->cour] = true;
        for (int i = 1; i <= params.nbClients; i++)
        {
            bestInsertClient[u.route->cour][i].whenLastCalculated = -1;
            bestInsertClientTW[u.route->cour][i].whenLastCalculated = -1;
        }
    }
    if (!bestInsertInitializedForRoute[v.route->cour])
    {
        bestInsertInitializedForRoute[v.route->cour] = true;
        for (int i = 1; i <= params.nbClients; i++)
        {
            bestInsertClient[v.route->cour][i].whenLastCalculated = -1;
            bestInsertClientTW[v.route->cour][i].whenLastCalculated = -1;
        }
    }

    // Preprocessing insertion costs
    if (withTW)
    {
        preprocessInsertionsWithTW(u.route, v.route);
        preprocessInsertionsWithTW(v.route, u.route);
    }
    else
    {
        preprocessInsertions(u.route, v.route);
        preprocessInsertions(v.route, u.route);
    }

    // Evaluating the moves
    for (auto *nodeU = u.route->depot->next; !nodeU->isDepot;
         nodeU = nodeU->next)
    {
        for (auto *nodeV = v.route->depot->next; !nodeV->isDepot;
             nodeV = nodeV->next)
        {
            // We cannot determine impact on time warp without adding too much
            // complexity (O(n^3) instead of O(n^2))
            int const loadPenU = penaltyExcessLoad(
                u.route->load + params.clients[nodeV->cour].demand
                - params.clients[nodeU->cour].demand);
            int const loadPenV = penaltyExcessLoad(
                v.route->load + params.clients[nodeU->cour].demand
                - params.clients[nodeV->cour].demand);
            int const deltaLoadPen = loadPenU + loadPenV
                                        - penaltyExcessLoad(u.route->load)
                                        - penaltyExcessLoad(v.route->load);
            int const deltaRemoval
                = withTW ? nodeU->deltaRemovalTW + nodeV->deltaRemovalTW
                         : nodeU->deltaRemoval + nodeV->deltaRemoval;

            // Quick filter: possibly early elimination of many SWAP* due to the
            // capacity constraints/penalties and bounds on insertion costs
            if (deltaLoadPen + deltaRemoval <= 0)
            {
                SwapStarElement swap;
                swap.U = nodeU;
                swap.V = nodeV;

                int extraV, extraU;
                if (withTW)
                {
                    // Evaluate reinsertion cost of U in the route of V where V
                    // has been removed
                    extraV = getCheapestInsertSimultRemovalWithTW(
                        nodeU, nodeV, swap.bestPositionU);

                    // Evaluate reinsertion cost of V in the route of U where U
                    // has been removed
                    extraU = getCheapestInsertSimultRemovalWithTW(
                        nodeV, nodeU, swap.bestPositionV);
                }
                else
                {
                    // Evaluate reinsertion cost of U in the route of V where V
                    // has been removed
                    extraV = getCheapestInsertSimultRemoval(
                        nodeU, nodeV, swap.bestPositionU);

                    // Evaluate reinsertion cost of V in the route of U where U
                    // has been removed
                    extraU = getCheapestInsertSimultRemoval(
                        nodeV, nodeU, swap.bestPositionV);
                }

                swap.moveCost = deltaLoadPen + deltaRemoval + extraU + extraV;

                if (swap.moveCost < bestSwap.moveCost)
                {
                    bestSwap = swap;
                    bestSwap.loadPenU = loadPenU;
                    bestSwap.loadPenV = loadPenV;
                }
            }
        }
    }

    if (!bestSwap.bestPositionU || !bestSwap.bestPositionV)
    {
        return false;
    }

    // Compute actual cost including TimeWarp penalty
    int costSuppU
        = params.dist(bestSwap.bestPositionV->cour, bestSwap.V->cour)
          - params.dist(bestSwap.U->prev->cour, bestSwap.U->cour)
          - params.dist(bestSwap.U->cour, bestSwap.U->next->cour);
    int costSuppV
        = params.dist(bestSwap.bestPositionU->cour, bestSwap.U->cour)
          - params.dist(bestSwap.V->prev->cour, bestSwap.V->cour)
          - params.dist(bestSwap.V->cour, bestSwap.V->next->cour);

    if (bestSwap.bestPositionV == bestSwap.U->prev)
    {
        // Insert in place of U
        costSuppU += params.dist(bestSwap.V->cour, bestSwap.U->next->cour);
    }
    else
    {
        costSuppU
            += params.dist(bestSwap.V->cour, bestSwap.bestPositionV->next->cour)
               + params.dist(bestSwap.U->prev->cour, bestSwap.U->next->cour)
               - params.dist(bestSwap.bestPositionV->cour,
                             bestSwap.bestPositionV->next->cour);
    }

    if (bestSwap.bestPositionU == bestSwap.V->prev)
    {
        // Insert in place of V
        costSuppV += params.dist(bestSwap.U->cour, bestSwap.V->next->cour);
    }
    else
    {
        costSuppV
            += params.dist(bestSwap.U->cour, bestSwap.bestPositionU->next->cour)
               + params.dist(bestSwap.V->prev->cour, bestSwap.V->next->cour)
               - params.dist(bestSwap.bestPositionU->cour,
                             bestSwap.bestPositionU->next->cour);
    }

    // It is not possible to have bestPositionU == V or bestPositionV == U, so
    // the positions are always strictly different
    if (bestSwap.bestPositionV->position == bestSwap.U->position - 1)
    {
        // Special case
        auto const routeUTwData
            = mergeTwDataRecursive(bestSwap.bestPositionV->prefixTwData,
                                   bestSwap.V->twData,
                                   bestSwap.U->next->postfixTwData);

        costSuppU += penaltyTimeWindows(routeUTwData);
    }
    else if (bestSwap.bestPositionV->position < bestSwap.U->position)
    {
        auto const routeUTwData = mergeTwDataRecursive(
            bestSwap.bestPositionV->prefixTwData,
            bestSwap.V->twData,
            getRouteSegmentTwData(bestSwap.bestPositionV->next,
                                  bestSwap.U->prev),
            bestSwap.U->next->postfixTwData);

        costSuppU += penaltyTimeWindows(routeUTwData);
    }
    else
    {
        auto const routeUTwData = mergeTwDataRecursive(
            bestSwap.U->prev->prefixTwData,
            getRouteSegmentTwData(bestSwap.U->next, bestSwap.bestPositionV),
            bestSwap.V->twData,
            bestSwap.bestPositionV->next->postfixTwData);

        costSuppU += penaltyTimeWindows(routeUTwData);
    }

    if (bestSwap.bestPositionU->position == bestSwap.V->position - 1)
    {
        // Special case
        auto const routeVTwData
            = mergeTwDataRecursive(bestSwap.bestPositionU->prefixTwData,
                                   bestSwap.U->twData,
                                   bestSwap.V->next->postfixTwData);

        costSuppV += penaltyTimeWindows(routeVTwData);
    }
    else if (bestSwap.bestPositionU->position < bestSwap.V->position)
    {
        auto const routeVTwData = mergeTwDataRecursive(
            bestSwap.bestPositionU->prefixTwData,
            bestSwap.U->twData,
            getRouteSegmentTwData(bestSwap.bestPositionU->next,
                                  bestSwap.V->prev),
            bestSwap.V->next->postfixTwData);

        costSuppV += penaltyTimeWindows(routeVTwData);
    }
    else
    {
        auto const routeVTwData = mergeTwDataRecursive(
            bestSwap.V->prev->prefixTwData,
            getRouteSegmentTwData(bestSwap.V->next, bestSwap.bestPositionU),
            bestSwap.U->twData,
            bestSwap.bestPositionU->next->postfixTwData);

        costSuppV += penaltyTimeWindows(routeVTwData);
    }

    costSuppU += bestSwap.loadPenU - u.route->penalty;
    costSuppV += bestSwap.loadPenV - v.route->penalty;

    if (costSuppU + costSuppV >= 0)
    {
        return false;
    }

    // Applying the best move in case of improvement
    insertNode(bestSwap.U, bestSwap.bestPositionU);
    insertNode(bestSwap.V, bestSwap.bestPositionV);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(u.route);
    updateRouteData(v.route);

    return true;
}

bool LocalSearch::RelocateStar()
{
    int bestCost = 0;
    Node *insertionPoint = nullptr;
    Node *nodeToInsert = nullptr;
    for (auto *nodeU = u.route->depot->next; !nodeU->isDepot;
         nodeU = nodeU->next)
    {
        u = {params, nodeU};

        const TimeWindowData routeUTwData = mergeTwDataRecursive(
            nodeU->prev->prefixTwData, u.nextNode->postfixTwData);
        int const costSuppU = params.dist(u.prevIndex, u.nextIndex)
                                 - params.dist(u.prevIndex, u.nodeIndex)
                                 - params.dist(u.nodeIndex, u.nextIndex)
                                 + penaltyExcessLoad(u.route->load - u.demand)
                                 + penaltyTimeWindows(routeUTwData)
                                 - u.route->penalty;

        for (Node *V = v.route->depot->next; !V->isDepot; V = V->next)
        {
            const TimeWindowData routeVTwData = mergeTwDataRecursive(
                V->prefixTwData, nodeU->twData, V->next->postfixTwData);
            int const costSuppV = params.dist(V->cour, u.nodeIndex)
                               + params.dist(u.nodeIndex, V->next->cour)
                               - params.dist(V->cour, V->next->cour)
                               + penaltyExcessLoad(v.route->load + u.demand)
                               + penaltyTimeWindows(routeVTwData)
                               - v.route->penalty;

            if (costSuppU + costSuppV < bestCost)
            {
                bestCost = costSuppU + costSuppV;
                insertionPoint = V;
                nodeToInsert = nodeU;
            }
        }
    }

    if (!insertionPoint)
    {
        return false;
    }

    u.route = nodeToInsert->route;
    insertNode(nodeToInsert, insertionPoint);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(u.route);
    updateRouteData(insertionPoint->route);

    return true;
}

int LocalSearch::getCheapestInsertSimultRemoval(Node *U,
                                                Node *V,
                                                Node *&bestPosition)
{
    ThreeBestInsert *myBestInsert = &bestInsertClient[V->route->cour][U->cour];
    bool found = false;

    // Finds the best insertion in the route such that V is not next or pred
    // (can only belong to the top three locations)
    bestPosition = myBestInsert->bestLocation[0];
    int bestCost = myBestInsert->bestCost[0];
    found = (bestPosition != V && bestPosition->next != V);
    if (!found && myBestInsert->bestLocation[1] != nullptr)
    {
        bestPosition = myBestInsert->bestLocation[1];
        bestCost = myBestInsert->bestCost[1];
        found = (bestPosition != V && bestPosition->next != V);
        if (!found && myBestInsert->bestLocation[2] != nullptr)
        {
            bestPosition = myBestInsert->bestLocation[2];
            bestCost = myBestInsert->bestCost[2];
            found = true;
        }
    }

    // Compute insertion in the place of V
    int deltaCost = params.dist(V->prev->cour, U->cour)
                    + params.dist(U->cour, V->next->cour)
                    - params.dist(V->prev->cour, V->next->cour);
    if (!found || deltaCost < bestCost)
    {
        bestPosition = V->prev;
        bestCost = deltaCost;
    }

    return bestCost;
}

int LocalSearch::getCheapestInsertSimultRemovalWithTW(Node *U,
                                                      Node *V,
                                                      Node *&bestPosition)
{
    ThreeBestInsert *myBestInsert
        = &bestInsertClientTW[V->route->cour][U->cour];
    bool found = false;

    // Finds the best insertion in the route such that V is not next or pred
    // (can only belong to the top three locations)
    bestPosition = myBestInsert->bestLocation[0];
    int bestCost = myBestInsert->bestCost[0];
    found = (bestPosition != V && bestPosition->next != V);
    if (!found && myBestInsert->bestLocation[1] != nullptr)
    {
        bestPosition = myBestInsert->bestLocation[1];
        bestCost = myBestInsert->bestCost[1];
        found = (bestPosition != V && bestPosition->next != V);
        if (!found && myBestInsert->bestLocation[2] != nullptr)
        {
            bestPosition = myBestInsert->bestLocation[2];
            bestCost = myBestInsert->bestCost[2];
            found = true;
        }
    }

    // Compute insertion in the place of V
    TimeWindowData twData = mergeTwDataRecursive(
        V->prev->prefixTwData, U->twData, V->next->postfixTwData);

    int deltaCost = params.dist(V->prev->cour, U->cour)
                       + params.dist(U->cour, V->next->cour)
                       - params.dist(V->prev->cour, V->next->cour)
                       + penaltyTimeWindows(twData)
                       - penaltyTimeWindows(V->route->twData);

    if (!found || deltaCost < bestCost)
    {
        bestPosition = V->prev;
        bestCost = deltaCost;
    }

    return bestCost;
}

void LocalSearch::preprocessInsertions(Route *R1, Route *R2)
{
    for (Node *U = R1->depot->next; !U->isDepot; U = U->next)
    {
        // Performs the preprocessing
        U->deltaRemoval = params.dist(U->prev->cour, U->next->cour)
                          - params.dist(U->prev->cour, U->cour)
                          - params.dist(U->cour, U->next->cour);
        auto &currentOption = bestInsertClient[R2->cour][U->cour];
        if (R2->whenLastModified > currentOption.whenLastCalculated)
        {
            currentOption = ThreeBestInsert();
            currentOption.whenLastCalculated = nbMoves;
            currentOption.bestCost[0]
                = params.dist(0, U->cour)
                  + params.dist(U->cour, R2->depot->next->cour)
                  - params.dist(0, R2->depot->next->cour);
            currentOption.bestLocation[0] = R2->depot;
            for (Node *V = R2->depot->next; !V->isDepot; V = V->next)
            {
                int deltaCost = params.dist(V->cour, U->cour)
                                + params.dist(U->cour, V->next->cour)
                                - params.dist(V->cour, V->next->cour);
                currentOption.add(deltaCost, V);
            }
        }
    }
}

void LocalSearch::preprocessInsertionsWithTW(Route *R1, Route *R2)
{
    for (Node *U = R1->depot->next; !U->isDepot; U = U->next)
    {
        // Performs the preprocessing
        // Note: when removing U and adding V to a route, the timewarp penalties
        // may interact, however in most cases it will hold that the reduced
        // timewarp from removing U + added timewarp from adding V will be
        // bigger than the actual delta timewarp such that assuming independence
        // gives a conservative estimate

        if (R1->isDeltaRemovalTWOutdated)
        {
            auto const twData = mergeTwDataRecursive(U->prev->prefixTwData,
                                                     U->next->postfixTwData);
            U->deltaRemovalTW = params.dist(U->prev->cour, U->next->cour)
                                - params.dist(U->prev->cour, U->cour)
                                - params.dist(U->cour, U->next->cour)
                                + penaltyTimeWindows(twData)
                                - penaltyTimeWindows(R1->twData);
        }
        auto &currentOption = bestInsertClientTW[R2->cour][U->cour];
        if (R2->whenLastModified > currentOption.whenLastCalculated)
        {
            currentOption = ThreeBestInsert();
            currentOption.whenLastCalculated = nbMoves;

            // Compute additional timewarp we get when inserting U in R2, this
            // may be actually less if we remove U but we ignore this to have a
            // conservative estimate
            auto twData = mergeTwDataRecursive(R2->depot->prefixTwData,
                                               U->twData,
                                               R2->depot->next->postfixTwData);

            int cost = params.dist(0, U->cour)
                       + params.dist(U->cour, R2->depot->next->cour)
                       - params.dist(0, R2->depot->next->cour)
                       + penaltyTimeWindows(twData)
                       - penaltyTimeWindows(R2->twData);

            currentOption.add(cost, R2->depot);

            for (Node *V = R2->depot->next; !V->isDepot; V = V->next)
            {
                twData = mergeTwDataRecursive(
                    V->prefixTwData, U->twData, V->next->postfixTwData);

                int deltaCost = params.dist(V->cour, U->cour)
                                + params.dist(U->cour, V->next->cour)
                                - params.dist(V->cour, V->next->cour)
                                + penaltyTimeWindows(twData)
                                - penaltyTimeWindows(R2->twData);

                currentOption.add(deltaCost, V);
            }
        }
    }

    R1->isDeltaRemovalTWOutdated = false;
}

LocalSearch::TimeWindowData LocalSearch::getEdgeTwData(Node *U, Node *V)
{
    // TODO this could be cached?
    return mergeTwDataRecursive(U->twData, V->twData);
}

LocalSearch::TimeWindowData LocalSearch::getRouteSegmentTwData(Node *U, Node *V)
{
    if (U->isDepot)
        return V->prefixTwData;
    if (V->isDepot)
        return U->postfixTwData;

    // Struct so this makes a copy
    TimeWindowData twData = U->twData;

    Node *mynode = U;
    const int targetPos = V->position;
    while (mynode != V)
    {
        if (mynode->isSeed && mynode->position + 4 <= targetPos)
        {
            twData = mergeTwDataRecursive(twData, mynode->toNextSeedTwD);
            mynode = mynode->nextSeed;
        }
        else
        {
            mynode = mynode->next;
            twData = mergeTwDataRecursive(twData, mynode->twData);
        }
    }
    return twData;
}

LocalSearch::TimeWindowData
LocalSearch::mergeTwDataRecursive(TimeWindowData const &twData1,
                                  TimeWindowData const &twData2) const
{
    int dist = params.dist(twData1.idxLast, twData2.idxFirst);
    int delta = twData1.duration - twData1.timeWarp + dist;
    int deltaWaitTime = std::max(twData2.twEarly - delta - twData1.twLate, 0);
    int deltaTimeWarp = std::max(twData1.twEarly + delta - twData2.twLate, 0);

    return {twData1.idxFirst,
            twData2.idxLast,
            twData1.duration + twData2.duration + dist + deltaWaitTime,
            twData1.timeWarp + twData2.timeWarp + deltaTimeWarp,
            std::max(twData2.twEarly - delta, twData1.twEarly) - deltaWaitTime,
            std::min(twData2.twLate - delta, twData1.twLate) + deltaTimeWarp,
            std::max(twData1.latestReleaseTime, twData2.latestReleaseTime)};
}

void LocalSearch::insertNode(Node *toInsert, Node *insertionPoint)
{
    toInsert->prev->next = toInsert->next;
    toInsert->next->prev = toInsert->prev;
    insertionPoint->next->prev = toInsert;
    toInsert->prev = insertionPoint;
    toInsert->next = insertionPoint->next;
    insertionPoint->next = toInsert;
    toInsert->route = insertionPoint->route;
}

void LocalSearch::swapNode(Node *U, Node *V)
{
    Node *myVPred = V->prev;
    Node *myVSuiv = V->next;
    Node *myUPred = U->prev;
    Node *myUSuiv = U->next;
    Route *myRouteU = U->route;
    Route *myRouteV = V->route;

    myUPred->next = V;
    myUSuiv->prev = V;
    myVPred->next = U;
    myVSuiv->prev = U;

    U->prev = myVPred;
    U->next = myVSuiv;
    V->prev = myUPred;
    V->next = myUSuiv;

    U->route = myRouteV;
    V->route = myRouteU;
}

void LocalSearch::updateRouteData(Route *myRoute)
{
    int myplace = 0;
    int myload = 0;
    int myReversalDistance = 0;
    int cumulatedX = 0;
    int cumulatedY = 0;

    Node *mynode = myRoute->depot;
    mynode->position = 0;
    mynode->cumulatedLoad = 0;
    mynode->cumulatedReversalDistance = 0;

    bool firstIt = true;
    TimeWindowData seedTwD = mynode->twData;
    Node *seedNode = nullptr;
    while (!mynode->isDepot || firstIt)
    {
        mynode = mynode->next;
        myplace++;
        mynode->position = myplace;
        myload += params.clients[mynode->cour].demand;
        myReversalDistance += params.dist(mynode->cour, mynode->prev->cour)
                              - params.dist(mynode->prev->cour, mynode->cour);
        mynode->cumulatedLoad = myload;
        mynode->cumulatedReversalDistance = myReversalDistance;
        mynode->prefixTwData
            = mergeTwDataRecursive(mynode->prev->prefixTwData, mynode->twData);
        mynode->isSeed = false;
        mynode->nextSeed = nullptr;
        if (!mynode->isDepot)
        {
            cumulatedX += params.clients[mynode->cour].x;
            cumulatedY += params.clients[mynode->cour].y;
            if (firstIt)
                myRoute->sector.initialize(params.clients[mynode->cour].angle);
            else
                myRoute->sector.extend(params.clients[mynode->cour].angle);

            if (myplace % 4 == 0)
            {
                if (seedNode != nullptr)
                {
                    seedNode->isSeed = true;
                    seedNode->toNextSeedTwD
                        = mergeTwDataRecursive(seedTwD, mynode->twData);
                    seedNode->nextSeed = mynode;
                }
                seedNode = mynode;
            }
            else if (myplace % 4 == 1)
                seedTwD = mynode->twData;
            else
                seedTwD = mergeTwDataRecursive(seedTwD, mynode->twData);
        }
        firstIt = false;
    }

    myRoute->load = myload;
    myRoute->twData = mynode->prefixTwData;
    myRoute->penalty
        = penaltyExcessLoad(myload) + penaltyTimeWindows(myRoute->twData);
    myRoute->nbCustomers = myplace - 1;
    // Remember "when" this route has been last modified (will be used to filter
    // unnecessary move evaluations)
    myRoute->whenLastModified = nbMoves;
    myRoute->isDeltaRemovalTWOutdated = true;

    // Time window data in reverse direction, mynode should be end depot now
    do
    {
        mynode = mynode->prev;
        mynode->postfixTwData
            = mergeTwDataRecursive(mynode->twData, mynode->next->postfixTwData);
    } while (!mynode->isDepot);

    if (myRoute->nbCustomers == 0)
    {
        myRoute->polarAngleBarycenter = 1.e30;
        emptyRoutes.insert(myRoute->cour);
    }
    else
    {
        myRoute->polarAngleBarycenter
            = atan2(cumulatedY / static_cast<double>(myRoute->nbCustomers)
                        - params.clients[0].y,
                    cumulatedX / static_cast<double>(myRoute->nbCustomers)
                        - params.clients[0].x);

        // Enforce minimum size of circle sector
        if (params.config.minCircleSectorSize > 0)
        {
            const int growSectorBy
                = (params.config.minCircleSectorSize
                   - CircleSector::positive_mod(myRoute->sector) + 1)
                  / 2;

            if (growSectorBy > 0)
            {
                myRoute->sector.extend(myRoute->sector.start - growSectorBy);
                myRoute->sector.extend(myRoute->sector.end + growSectorBy);
            }
        }
        emptyRoutes.erase(myRoute->cour);
    }
}

void LocalSearch::loadIndividual(Individual const &indiv)
{
    emptyRoutes.clear();
    nbMoves = 0;
    TimeWindowData depotTwData = {0,
                                  0,
                                  0,
                                  0,
                                  params.clients[0].twEarly,
                                  params.clients[0].twLate,
                                  params.clients[0].releaseTime};

    // Initializing time window data (before loop since it is needed in update
    // route)
    for (int i = 1; i <= params.nbClients; i++)
    {
        TimeWindowData *myTwData = &clients[i].twData;
        myTwData->idxFirst = i;
        myTwData->idxLast = i;
        myTwData->duration = params.clients[i].servDur;
        myTwData->twEarly = params.clients[i].twEarly;
        myTwData->twLate = params.clients[i].twLate;
        myTwData->latestReleaseTime = params.clients[i].releaseTime;
    }

    auto const &routesIndiv = indiv.getRoutes();

    for (int r = 0; r < params.nbVehicles; r++)
    {
        Node *myDepot = &depots[r];
        Node *myDepotFin = &depotsEnd[r];
        Route *myRoute = &routes[r];
        myDepot->prev = myDepotFin;
        myDepotFin->next = myDepot;
        if (!routesIndiv[r].empty())
        {
            Node *myClient = &clients[routesIndiv[r][0]];
            myClient->route = myRoute;
            myClient->prev = myDepot;
            myDepot->next = myClient;
            for (int i = 1; i < static_cast<int>(routesIndiv[r].size()); i++)
            {
                Node *myClientPred = myClient;
                myClient = &clients[routesIndiv[r][i]];
                myClient->prev = myClientPred;
                myClientPred->next = myClient;
                myClient->route = myRoute;
            }
            myClient->next = myDepotFin;
            myDepotFin->prev = myClient;
        }
        else
        {
            myDepot->next = myDepotFin;
            myDepotFin->prev = myDepot;
        }

        myDepot->twData = depotTwData;
        myDepot->prefixTwData = depotTwData;
        myDepot->postfixTwData = depotTwData;
        myDepot->isSeed = false;

        myDepotFin->twData = depotTwData;
        myDepotFin->prefixTwData = depotTwData;
        myDepotFin->postfixTwData = depotTwData;
        myDepotFin->isSeed = false;

        updateRouteData(&routes[r]);
        routes[r].whenLastTestedLargeNb = -1;
        bestInsertInitializedForRoute[r] = false;
    }

    for (int i = 1; i <= params.nbClients;
         i++)  // Initializing memory structures
        clients[i].whenLastTestedRI = -1;
}

Individual LocalSearch::exportIndividual()
{
    std::vector<std::pair<double, int>> routePolarAngles;
    routePolarAngles.reserve(params.nbVehicles);

    for (int r = 0; r < params.nbVehicles; r++)
        routePolarAngles.emplace_back(routes[r].polarAngleBarycenter, r);

    // empty indivRoutes have a polar angle of 1.e30, and therefore will always
    // appear at the end
    std::sort(routePolarAngles.begin(), routePolarAngles.end());

    std::vector<int> tour;
    tour.reserve(params.nbClients);

    std::vector<std::vector<int>> indivRoutes(params.nbVehicles);

    for (int r = 0; r < params.nbVehicles; r++)
    {
        Node *node = depots[routePolarAngles[r].second].next;

        while (!node->isDepot)
        {
            tour.push_back(node->cour);
            indivRoutes[r].push_back(node->cour);
            node = node->next;
        }
    }

    return {&params, tour, indivRoutes};
}

LocalSearch::LocalSearch(Params &params, XorShift128 &rng)
    : params(params),
      rng(rng),
      orderNodes(params.nbClients),
      orderRoutes(params.nbVehicles)
{
    clients = std::vector<Node>(params.nbClients + 1);
    routes = std::vector<Route>(params.nbVehicles);
    depots = std::vector<Node>(params.nbVehicles);
    depotsEnd = std::vector<Node>(params.nbVehicles);
    bestInsertInitializedForRoute = std::vector<bool>(params.nbVehicles, false);
    bestInsertClient = std::vector<std::vector<ThreeBestInsert>>(
        params.nbVehicles, std::vector<ThreeBestInsert>(params.nbClients + 1));
    bestInsertClientTW = std::vector<std::vector<ThreeBestInsert>>(
        params.nbVehicles, std::vector<ThreeBestInsert>(params.nbClients + 1));

    for (int i = 0; i <= params.nbClients; i++)
    {
        clients[i].cour = i;
        clients[i].isDepot = false;
    }

    for (int i = 0; i < params.nbVehicles; i++)
    {
        routes[i].cour = i;
        routes[i].depot = &depots[i];
        depots[i].cour = 0;
        depots[i].isDepot = true;
        depots[i].route = &routes[i];
        depotsEnd[i].cour = 0;
        depotsEnd[i].isDepot = true;
        depotsEnd[i].route = &routes[i];
    }

    std::iota(orderNodes.begin(), orderNodes.end(), 1);
    std::iota(orderRoutes.begin(), orderRoutes.end(), 0);
}
