#include "LocalSearch.h"

#include "CircleSector.h"
#include "Individual.h"
#include "Params.h"

#include <cmath>
#include <numeric>
#include <vector>

void LocalSearch::operator()(Individual &indiv,
                             int excessCapacityPenalty,
                             int timeWarpPenalty)
{
    penalties = {&params, excessCapacityPenalty, timeWarpPenalty};

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

    bool searchCompleted = false;  // No further improving move?
    int nbMoves = 0;               // Operator (RI and SWAP*) counter

    Node *nodeU;
    Node *nodeV;
    Node *nodeX;
    Node *nodeY;
    Route *routeU;
    Route *routeV;

    for (int step = 0; !searchCompleted; ++step)
    {
        if (step > 1)                // At least two loops as some moves with
            searchCompleted = true;  // empty routes are not done in the first

        /* ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY RESTRICTION */
        for (int posU = 0; posU < params.nbClients; posU++)
        {
            nodeU = &clients[orderNodes[posU]];
            int lastTestRINodeU = nodeU->whenLastTestedRI;
            nodeU->whenLastTestedRI = nbMoves;

            // Randomizing the order of the neighborhoods within this loop does
            // not matter much as we are already randomizing the order of the
            // node pairs (and it's not very common to find improving moves of
            // different types for the same node pair)
            for (auto const &v : params.getNeighboursOf(nodeU->cour))
            {
                routeU = nodeU->route;
                nodeX = nodeU->next;

                nodeV = &clients[v];
                routeV = nodeV->route;
                nodeY = nodeV->next;

                if (step == 0
                    || std::max(nodeU->route->whenLastModified,
                                nodeV->route->whenLastModified)
                           > lastTestRINodeU)  // only evaluate moves involving
                                               // routes that have been
                                               // modified since last move
                                               // evaluations for nodeU
                {
                    if (MoveSingleClient(nbMoves,
                                         searchCompleted,
                                         nodeU,
                                         nodeV,
                                         nodeX,
                                         nodeY,
                                         routeU,
                                         routeV))
                        continue;  // RELOCATE
                    if (MoveTwoClients(nbMoves,
                                       searchCompleted,
                                       nodeU,
                                       nodeV,
                                       nodeX,
                                       nodeY,
                                       routeU,
                                       routeV))
                        continue;  // RELOCATE
                    if (MoveTwoClientsReversed(nbMoves,
                                               searchCompleted,
                                               nodeU,
                                               nodeV,
                                               nodeX,
                                               nodeY,
                                               routeU,
                                               routeV))
                        continue;  // RELOCATE
                    if (SwapTwoSingleClients(nbMoves,
                                             searchCompleted,
                                             nodeU,
                                             nodeV,
                                             nodeX,
                                             nodeY,
                                             routeU,
                                             routeV))
                        continue;  // SWAP
                    if (SwapTwoClientsForOne(nbMoves,
                                             searchCompleted,
                                             nodeU,
                                             nodeV,
                                             nodeX,
                                             nodeY,
                                             routeU,
                                             routeV))
                        continue;  // SWAP
                    if (SwapTwoClientPairs(nbMoves,
                                           searchCompleted,
                                           nodeU,
                                           nodeV,
                                           nodeX,
                                           nodeY,
                                           routeU,
                                           routeV))
                        continue;  // SWAP
                    if (TwoOptBetweenTrips(nbMoves,
                                           searchCompleted,
                                           nodeU,
                                           nodeV,
                                           nodeX,
                                           nodeY,
                                           routeU,
                                           routeV))
                        continue;  // 2-OPT*
                    if (TwoOptWithinTrip(nbMoves,
                                         searchCompleted,
                                         nodeU,
                                         nodeV,
                                         nodeX,
                                         nodeY,
                                         routeU,
                                         routeV))
                        continue;  // 2-OPT

                    // Trying moves that insert nodeU directly after the depot
                    if (nodeV->prev->isDepot)
                    {
                        nodeV = nodeV->prev;
                        routeV = nodeV->route;
                        nodeY = nodeV->next;

                        if (MoveSingleClient(nbMoves,
                                             searchCompleted,
                                             nodeU,
                                             nodeV,
                                             nodeX,
                                             nodeY,
                                             routeU,
                                             routeV))
                            continue;  // RELOCATE
                        if (MoveTwoClients(nbMoves,
                                           searchCompleted,
                                           nodeU,
                                           nodeV,
                                           nodeX,
                                           nodeY,
                                           routeU,
                                           routeV))
                            continue;  // RELOCATE
                        if (MoveTwoClientsReversed(nbMoves,
                                                   searchCompleted,
                                                   nodeU,
                                                   nodeV,
                                                   nodeX,
                                                   nodeY,
                                                   routeU,
                                                   routeV))
                            continue;  // RELOCATE
                        if (TwoOptBetweenTrips(nbMoves,
                                               searchCompleted,
                                               nodeU,
                                               nodeV,
                                               nodeX,
                                               nodeY,
                                               routeU,
                                               routeV))
                            continue;  // 2-OPT*
                    }
                }
            }

            /* MOVES INVOLVING AN EMPTY ROUTE -- NOT TESTED IN THE FIRST LOOP TO
             * AVOID INCREASING TOO MUCH THE FLEET SIZE */
            if (step > 0 && !emptyRoutes.empty())
            {
                routeU = nodeU->route;
                nodeX = nodeU->next;

                nodeV = routes[*emptyRoutes.begin()].depot;
                routeV = nodeV->route;
                nodeY = nodeV->next;

                if (MoveSingleClient(nbMoves,
                                     searchCompleted,
                                     nodeU,
                                     nodeV,
                                     nodeX,
                                     nodeY,
                                     routeU,
                                     routeV))
                    continue;  // RELOCATE
                if (MoveTwoClients(nbMoves,
                                   searchCompleted,
                                   nodeU,
                                   nodeV,
                                   nodeX,
                                   nodeY,
                                   routeU,
                                   routeV))
                    continue;  // RELOCATE
                if (MoveTwoClientsReversed(nbMoves,
                                           searchCompleted,
                                           nodeU,
                                           nodeV,
                                           nodeX,
                                           nodeY,
                                           routeU,
                                           routeV))
                    continue;  // RELOCATE
                if (TwoOptBetweenTrips(nbMoves,
                                       searchCompleted,
                                       nodeU,
                                       nodeV,
                                       nodeX,
                                       nodeY,
                                       routeU,
                                       routeV))
                    continue;  // 2-OPT*
            }
        }

        /* (SWAP*) MOVES LIMITED TO ROUTE PAIRS WHOSE CIRCLE SECTORS OVERLAP */
        if (searchCompleted && shouldIntensify)
        {
            for (int rU = 0; rU < params.nbVehicles; rU++)
            {
                routeU = &routes[orderRoutes[rU]];
                if (routeU->nbCustomers == 0)
                    continue;

                int lastTestLargeNbRouteU = routeU->whenLastTestedLargeNb;
                routeU->whenLastTestedLargeNb = nbMoves;
                for (int rV = 0; rV < params.nbVehicles; rV++)
                {
                    routeV = &routes[orderRoutes[rV]];
                    if (routeV->nbCustomers == 0
                        || routeU->cour >= routeV->cour)
                        continue;

                    if (step > 0
                        && std::max(routeU->whenLastModified,
                                    routeV->whenLastModified)
                               <= lastTestLargeNbRouteU)
                        continue;

                    if (!CircleSector::overlap(
                            routeU->sector,
                            routeV->sector,
                            params.config.circleSectorOverlapTolerance))
                        continue;

                    if (RelocateStar(nbMoves,
                                     searchCompleted,
                                     nodeU,
                                     nodeV,
                                     nodeX,
                                     nodeY,
                                     routeU,
                                     routeV))
                        continue;
                    if (swapStar(false,
                                 nbMoves,
                                 searchCompleted,
                                 nodeU,
                                 nodeV,
                                 nodeX,
                                 nodeY,
                                 routeU,
                                 routeV))
                        continue;
                    if (swapStar(true,
                                 nbMoves,
                                 searchCompleted,
                                 nodeU,
                                 nodeV,
                                 nodeX,
                                 nodeY,
                                 routeU,
                                 routeV))
                        continue;
                }
            }
        }
    }
}

bool LocalSearch::MoveSingleClient(int &nbMoves,
                                   bool &searchCompleted,
                                   Node *nodeU,
                                   Node *nodeV,
                                   Node *nodeX,
                                   Node *nodeY,
                                   Route *routeU,
                                   Route *routeV)
{
    // If U already comes directly after V, this move has no effect
    if (nodeU->cour == nodeY->cour)
        return false;

    int costSuppU = params.dist(nodeU->prev->cour, nodeX->cour)
                    - params.dist(nodeU->prev->cour, nodeU->cour)
                    - params.dist(nodeU->cour, nodeX->cour);
    int costSuppV = params.dist(nodeV->cour, nodeU->cour)
                    + params.dist(nodeU->cour, nodeY->cour)
                    - params.dist(nodeV->cour, nodeY->cour);

    if (routeU != routeV)
    {
        if (routeU->load <= params.vehicleCapacity
            && routeU->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = mergeTwDataRecursive(nodeU->prev->prefixTwData,
                                                 nodeX->postfixTwData);
        auto routeVTwData = mergeTwDataRecursive(
            nodeV->prefixTwData, nodeU->twData, nodeY->postfixTwData);

        costSuppU
            += penalties.load(routeU->load - params.clients[nodeU->cour].demand)
               + penalties.timeWarp(routeUTwData) - routeU->penalty;

        costSuppV
            += penalties.load(routeV->load + params.clients[nodeU->cour].demand)
               + penalties.timeWarp(routeVTwData) - routeV->penalty;
    }
    else
    {
        if (routeU->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Move within the same route
        if (nodeU->position < nodeV->position)
        {
            // Edge case V directly after U, so X == V, this works
            // start - ... - UPrev - X - ... - V - U - Y - ... - end
            auto const routeUTwData
                = mergeTwDataRecursive(nodeU->prev->prefixTwData,
                                       getRouteSegmentTwData(nodeX, nodeV),
                                       nodeU->twData,
                                       nodeY->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - U - Y - ... - UPrev - X - ... - end
            auto const routeUTwData = mergeTwDataRecursive(
                nodeV->prefixTwData,
                nodeU->twData,
                getRouteSegmentTwData(nodeY, nodeU->prev),
                nodeX->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(routeU->load) - routeU->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    insertNode(nodeU, nodeV);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU, nbMoves);
    if (routeU != routeV)
        updateRouteData(routeV, nbMoves);

    return true;
}

bool LocalSearch::MoveTwoClients(int &nbMoves,
                                 bool &searchCompleted,
                                 Node *nodeU,
                                 Node *nodeV,
                                 Node *nodeX,
                                 Node *nodeY,
                                 Route *routeU,
                                 Route *routeV)
{
    if (nodeU == nodeY || nodeV == nodeX || nodeX->isDepot)
        return false;

    int costSuppU = params.dist(nodeU->prev->cour, nodeX->next->cour)
                    - params.dist(nodeU->prev->cour, nodeU->cour)
                    - params.dist(nodeX->cour, nodeX->next->cour);
    int costSuppV = params.dist(nodeV->cour, nodeU->cour)
                    + params.dist(nodeX->cour, nodeY->cour)
                    - params.dist(nodeV->cour, nodeY->cour);

    if (routeU != routeV)
    {
        if (routeU->load <= params.vehicleCapacity
            && routeU->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = mergeTwDataRecursive(nodeU->prev->prefixTwData,
                                                 nodeX->next->postfixTwData);
        auto routeVTwData = mergeTwDataRecursive(nodeV->prefixTwData,
                                                 getEdgeTwData(nodeU, nodeX),
                                                 nodeY->postfixTwData);

        costSuppU
            += penalties.load(routeU->load - params.clients[nodeU->cour].demand
                              - params.clients[nodeX->cour].demand)
               + penalties.timeWarp(routeUTwData) - routeU->penalty;

        costSuppV
            += penalties.load(routeV->load + params.clients[nodeU->cour].demand
                              + params.clients[nodeX->cour].demand)
               + penalties.timeWarp(routeVTwData) - routeV->penalty;
    }
    else
    {
        if (routeU->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Move within the same route
        if (nodeU->position < nodeV->position)
        {
            // Edge case V directly after U, so X == V is excluded, V directly
            // after X so XNext == V works start - ... - UPrev - XNext - ... - V
            // - U - X - Y - ... - end
            auto const routeUTwData = mergeTwDataRecursive(
                nodeU->prev->prefixTwData,
                getRouteSegmentTwData(nodeX->next, nodeV),
                getEdgeTwData(nodeU, nodeX),
                nodeY->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - U - X - Y - ... - UPrev - XNext - ...
            // - end
            auto const routeUTwData = mergeTwDataRecursive(
                nodeV->prefixTwData,
                getEdgeTwData(nodeU, nodeX),
                getRouteSegmentTwData(nodeY, nodeU->prev),
                nodeX->next->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(routeU->load) - routeU->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    insertNode(nodeU, nodeV);
    insertNode(nodeX, nodeU);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU, nbMoves);
    if (routeU != routeV)
        updateRouteData(routeV, nbMoves);

    return true;
}

bool LocalSearch::MoveTwoClientsReversed(int &nbMoves,
                                         bool &searchCompleted,
                                         Node *nodeU,
                                         Node *nodeV,
                                         Node *nodeX,
                                         Node *nodeY,
                                         Route *routeU,
                                         Route *routeV)
{
    if (nodeU == nodeY || nodeX == nodeV || nodeX->isDepot)
        return false;

    int costSuppU = params.dist(nodeU->prev->cour, nodeX->next->cour)
                    - params.dist(nodeU->prev->cour, nodeU->cour)
                    - params.dist(nodeU->cour, nodeX->cour)
                    - params.dist(nodeX->cour, nodeX->next->cour);
    int costSuppV = params.dist(nodeV->cour, nodeX->cour)
                    + params.dist(nodeX->cour, nodeU->cour)
                    + params.dist(nodeU->cour, nodeY->cour)
                    - params.dist(nodeV->cour, nodeY->cour);

    if (routeU != routeV)
    {
        if (routeU->load <= params.vehicleCapacity
            && routeU->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = mergeTwDataRecursive(nodeU->prev->prefixTwData,
                                                 nodeX->next->postfixTwData);
        auto routeVTwData = mergeTwDataRecursive(nodeV->prefixTwData,
                                                 getEdgeTwData(nodeX, nodeU),
                                                 nodeY->postfixTwData);

        costSuppU
            += penalties.load(routeU->load - params.clients[nodeU->cour].demand
                              - params.clients[nodeX->cour].demand)
               + penalties.timeWarp(routeUTwData) - routeU->penalty;

        costSuppV
            += penalties.load(routeV->load + params.clients[nodeU->cour].demand
                              + params.clients[nodeX->cour].demand)
               + penalties.timeWarp(routeVTwData) - routeV->penalty;
    }
    else
    {
        if (routeU->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Move within the same route
        if (nodeU->position < nodeV->position)
        {
            // Edge case V directly after U, so X == V is excluded, V directly
            // after X so XNext == V works start - ... - UPrev - XNext - ... - V
            // - X - U - Y - ... - end
            auto const routeUTwData = mergeTwDataRecursive(
                nodeU->prev->prefixTwData,
                getRouteSegmentTwData(nodeX->next, nodeV),
                getEdgeTwData(nodeX, nodeU),
                nodeY->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - X - U - Y - ... - UPrev - XNext - ...
            // - end
            auto const routeUTwData = mergeTwDataRecursive(
                nodeV->prefixTwData,
                getEdgeTwData(nodeX, nodeU),
                getRouteSegmentTwData(nodeY, nodeU->prev),
                nodeX->next->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(routeU->load) - routeU->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    insertNode(nodeX, nodeV);
    insertNode(nodeU, nodeX);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU, nbMoves);
    if (routeU != routeV)
        updateRouteData(routeV, nbMoves);

    return true;
}

bool LocalSearch::SwapTwoSingleClients(int &nbMoves,
                                       bool &searchCompleted,
                                       Node *nodeU,
                                       Node *nodeV,
                                       Node *nodeX,
                                       Node *nodeY,
                                       Route *routeU,
                                       Route *routeV)
{
    if (nodeU->cour >= nodeV->cour)
        return false;

    if (nodeU->cour == nodeV->prev->cour || nodeU->cour == nodeY->cour)
        return false;

    int costSuppU = params.dist(nodeU->prev->cour, nodeV->cour)
                    + params.dist(nodeV->cour, nodeX->cour)
                    - params.dist(nodeU->prev->cour, nodeU->cour)
                    - params.dist(nodeU->cour, nodeX->cour);
    int costSuppV = params.dist(nodeV->prev->cour, nodeU->cour)
                    + params.dist(nodeU->cour, nodeY->cour)
                    - params.dist(nodeV->prev->cour, nodeV->cour)
                    - params.dist(nodeV->cour, nodeY->cour);

    if (routeU != routeV)
    {
        if (routeU->load <= params.vehicleCapacity
            && routeU->twData.timeWarp == 0
            && routeV->load <= params.vehicleCapacity
            && routeV->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = mergeTwDataRecursive(
            nodeU->prev->prefixTwData, nodeV->twData, nodeX->postfixTwData);
        auto routeVTwData = mergeTwDataRecursive(
            nodeV->prev->prefixTwData, nodeU->twData, nodeY->postfixTwData);

        costSuppU
            += penalties.load(routeU->load + params.clients[nodeV->cour].demand
                              - params.clients[nodeU->cour].demand)
               + penalties.timeWarp(routeUTwData) - routeU->penalty;

        costSuppV
            += penalties.load(routeV->load + params.clients[nodeU->cour].demand
                              - params.clients[nodeV->cour].demand)
               + penalties.timeWarp(routeVTwData) - routeV->penalty;
    }
    else
    {
        if (routeU->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Swap within the same route
        if (nodeU->position < nodeV->position)
        {
            // Edge case V directly after U, so X == V is excluded, V directly
            // after X so XNext == V works start - ... - UPrev - V - X - ... -
            // VPrev - U - Y - ... - end
            auto const routeUTwData = mergeTwDataRecursive(
                nodeU->prev->prefixTwData,
                nodeV->twData,
                getRouteSegmentTwData(nodeX, nodeV->prev),
                nodeU->twData,
                nodeY->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - VPrev - U - Y - ... - UPrev - V - X - ...
            // - end
            auto const routeUTwData = mergeTwDataRecursive(
                nodeV->prev->prefixTwData,
                nodeU->twData,
                getRouteSegmentTwData(nodeY, nodeU->prev),
                nodeV->twData,
                nodeX->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(routeU->load) - routeU->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    swapNode(nodeU, nodeV);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU, nbMoves);
    if (routeU != routeV)
        updateRouteData(routeV, nbMoves);

    return true;
}

bool LocalSearch::SwapTwoClientsForOne(int &nbMoves,
                                       bool &searchCompleted,
                                       Node *nodeU,
                                       Node *nodeV,
                                       Node *nodeX,
                                       Node *nodeY,
                                       Route *routeU,
                                       Route *routeV)
{
    if (nodeU == nodeV->prev || nodeX == nodeV->prev || nodeU == nodeY
        || nodeX->isDepot)
        return false;

    int costSuppU = params.dist(nodeU->prev->cour, nodeV->cour)
                    + params.dist(nodeV->cour, nodeX->next->cour)
                    - params.dist(nodeU->prev->cour, nodeU->cour)
                    - params.dist(nodeX->cour, nodeX->next->cour);
    int costSuppV = params.dist(nodeV->prev->cour, nodeU->cour)
                    + params.dist(nodeX->cour, nodeY->cour)
                    - params.dist(nodeV->prev->cour, nodeV->cour)
                    - params.dist(nodeV->cour, nodeY->cour);

    if (routeU != routeV)
    {
        if (routeU->load <= params.vehicleCapacity
            && routeU->twData.timeWarp == 0
            && routeV->load <= params.vehicleCapacity
            && routeV->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = mergeTwDataRecursive(nodeU->prev->prefixTwData,
                                                 nodeV->twData,
                                                 nodeX->next->postfixTwData);
        auto routeVTwData = mergeTwDataRecursive(nodeV->prev->prefixTwData,
                                                 getEdgeTwData(nodeU, nodeX),
                                                 nodeY->postfixTwData);

        costSuppU
            += penalties.load(routeU->load + params.clients[nodeV->cour].demand
                              - params.clients[nodeU->cour].demand
                              - params.clients[nodeX->cour].demand)
               + penalties.timeWarp(routeUTwData) - routeU->penalty;

        costSuppV
            += penalties.load(routeV->load + params.clients[nodeU->cour].demand
                              + params.clients[nodeX->cour].demand
                              - params.clients[nodeV->cour].demand)
               + penalties.timeWarp(routeVTwData) - routeV->penalty;
    }
    else
    {
        if (routeU->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Swap within the same route
        if (nodeU->position < nodeV->position)
        {
            // start - ... - UPrev - V - XNext - ... - VPrev - U - X - Y - ... -
            // end
            auto const routeUTwData = mergeTwDataRecursive(
                nodeU->prev->prefixTwData,
                nodeV->twData,
                getRouteSegmentTwData(nodeX->next, nodeV->prev),
                getEdgeTwData(nodeU, nodeX),
                nodeY->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // start - ... - VPrev - U - X - Y - ... - UPrev - V - XNext - ... -
            // end
            auto const routeUTwData = mergeTwDataRecursive(
                nodeV->prev->prefixTwData,
                getEdgeTwData(nodeU, nodeX),
                getRouteSegmentTwData(nodeY, nodeU->prev),
                nodeV->twData,
                nodeX->next->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(routeU->load) - routeU->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    // Note: next two lines are a bit inefficient but we only update
    // occasionally and updateRouteData is much more costly anyway, efficient
    // checks are more important
    swapNode(nodeU, nodeV);
    insertNode(nodeX, nodeU);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU, nbMoves);
    if (routeU != routeV)
        updateRouteData(routeV, nbMoves);

    return true;
}

bool LocalSearch::SwapTwoClientPairs(int &nbMoves,
                                     bool &searchCompleted,
                                     Node *nodeU,
                                     Node *nodeV,
                                     Node *nodeX,
                                     Node *nodeY,
                                     Route *routeU,
                                     Route *routeV)
{
    if (nodeU->cour >= nodeV->cour)
        return false;

    if (nodeX->isDepot || nodeY->isDepot || nodeY == nodeU->prev
        || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next)
        return false;

    int costSuppU = params.dist(nodeU->prev->cour, nodeV->cour)
                    + params.dist(nodeY->cour, nodeX->next->cour)
                    - params.dist(nodeU->prev->cour, nodeU->cour)
                    - params.dist(nodeX->cour, nodeX->next->cour);
    int costSuppV = params.dist(nodeV->prev->cour, nodeU->cour)
                    + params.dist(nodeX->cour, nodeY->next->cour)
                    - params.dist(nodeV->prev->cour, nodeV->cour)
                    - params.dist(nodeY->cour, nodeY->next->cour);

    if (routeU != routeV)
    {
        if (routeU->load <= params.vehicleCapacity
            && routeU->twData.timeWarp == 0
            && routeV->load <= params.vehicleCapacity
            && routeV->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = mergeTwDataRecursive(nodeU->prev->prefixTwData,
                                                 getEdgeTwData(nodeV, nodeY),
                                                 nodeX->next->postfixTwData);
        auto routeVTwData = mergeTwDataRecursive(nodeV->prev->prefixTwData,
                                                 getEdgeTwData(nodeU, nodeX),
                                                 nodeY->next->postfixTwData);

        costSuppU
            += penalties.load(routeU->load + params.clients[nodeV->cour].demand
                              + params.clients[nodeY->cour].demand
                              - params.clients[nodeU->cour].demand
                              - params.clients[nodeX->cour].demand)
               + penalties.timeWarp(routeUTwData) - routeU->penalty;

        costSuppV
            += penalties.load(routeV->load + params.clients[nodeU->cour].demand
                              + params.clients[nodeX->cour].demand
                              - params.clients[nodeV->cour].demand
                              - params.clients[nodeY->cour].demand)
               + penalties.timeWarp(routeVTwData) - routeV->penalty;
    }
    else
    {
        if (routeU->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Swap within the same route
        if (nodeU->position < nodeV->position)
        {
            // start - ... - UPrev - V - Y - XNext - ... - VPrev - U - X - YNext
            // - ... - end
            auto const routeUTwData = mergeTwDataRecursive(
                nodeU->prev->prefixTwData,
                getEdgeTwData(nodeV, nodeY),
                getRouteSegmentTwData(nodeX->next, nodeV->prev),
                getEdgeTwData(nodeU, nodeX),
                nodeY->next->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // start - ... - VPrev - U - X - YNext - ... - UPrev - V - Y - XNext
            // - ... - end
            auto const routeUTwData = mergeTwDataRecursive(
                nodeV->prev->prefixTwData,
                getEdgeTwData(nodeU, nodeX),
                getRouteSegmentTwData(nodeY->next, nodeU->prev),
                getEdgeTwData(nodeV, nodeY),
                nodeX->next->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(routeU->load) - routeU->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    swapNode(nodeU, nodeV);
    swapNode(nodeX, nodeY);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU, nbMoves);
    if (routeU != routeV)
        updateRouteData(routeV, nbMoves);

    return true;
}

bool LocalSearch::TwoOptWithinTrip(int &nbMoves,
                                   bool &searchCompleted,
                                   Node *nodeU,
                                   Node *nodeV,
                                   Node *nodeX,
                                   Node *nodeY,
                                   Route *routeU,
                                   Route *routeV)
{
    if (routeU != routeV)
        return false;

    if (nodeU->position >= nodeV->position - 1)
        return false;

    int cost = params.dist(nodeU->cour, nodeV->cour)
               + params.dist(nodeX->cour, nodeY->cour)
               - params.dist(nodeU->cour, nodeX->cour)
               - params.dist(nodeV->cour, nodeY->cour)
               + nodeV->cumulatedReversalDistance
               - nodeX->cumulatedReversalDistance;

    if (routeU->twData.timeWarp == 0 && cost >= 0)
    {
        return false;
    }

    TimeWindowData routeTwData = nodeU->prefixTwData;
    Node *itRoute = nodeV;
    while (itRoute != nodeU)
    {
        routeTwData = mergeTwDataRecursive(routeTwData, itRoute->twData);
        itRoute = itRoute->prev;
    }
    routeTwData = mergeTwDataRecursive(routeTwData, nodeY->postfixTwData);

    // Compute new total penalty
    cost += penalties.load(routeU->load) + penalties.timeWarp(routeTwData)
            - routeU->penalty;

    if (cost >= 0)
    {
        return false;
    }

    itRoute = nodeV;
    Node *insertionPoint = nodeU;
    while (itRoute != nodeX)  // No need to move x, we pivot around it
    {
        Node *current = itRoute;
        itRoute = itRoute->prev;
        insertNode(current, insertionPoint);
        insertionPoint = current;
    }

    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU, nbMoves);

    return true;
}

bool LocalSearch::TwoOptBetweenTrips(int &nbMoves,
                                     bool &searchCompleted,
                                     Node *nodeU,
                                     Node *nodeV,
                                     Node *nodeX,
                                     Node *nodeY,
                                     Route *routeU,
                                     Route *routeV)
{
    if (routeU->cour >= routeV->cour)
        return false;

    int costSuppU = params.dist(nodeU->cour, nodeY->cour)
                    - params.dist(nodeU->cour, nodeX->cour);
    int costSuppV = params.dist(nodeV->cour, nodeX->cour)
                    - params.dist(nodeV->cour, nodeY->cour);

    if (routeU->load <= params.vehicleCapacity && routeU->twData.timeWarp == 0
        && routeV->load <= params.vehicleCapacity
        && routeV->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
    {
        return false;
    }

    auto routeUTwData
        = mergeTwDataRecursive(nodeU->prefixTwData, nodeY->postfixTwData);
    auto routeVTwData
        = mergeTwDataRecursive(nodeV->prefixTwData, nodeX->postfixTwData);

    costSuppU += penalties.load(nodeU->cumulatedLoad + routeV->load
                                - nodeV->cumulatedLoad)
                 + penalties.timeWarp(routeUTwData) - routeU->penalty;

    costSuppV += penalties.load(nodeV->cumulatedLoad + routeU->load
                                - nodeU->cumulatedLoad)
                 + penalties.timeWarp(routeVTwData) - routeV->penalty;

    if (costSuppU + costSuppV >= 0)
        return false;

    Node *itRouteV = nodeY;
    Node *insertLocation = nodeU;
    while (!itRouteV->isDepot)
    {
        Node *current = itRouteV;
        itRouteV = itRouteV->next;
        insertNode(current, insertLocation);
        insertLocation = current;
    }

    Node *itRouteU = nodeX;
    insertLocation = nodeV;
    while (!itRouteU->isDepot)
    {
        Node *current = itRouteU;
        itRouteU = itRouteU->next;
        insertNode(current, insertLocation);
        insertLocation = current;
    }

    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU, nbMoves);
    updateRouteData(routeV, nbMoves);

    return true;
}

bool LocalSearch::swapStar(bool const withTW,
                           int &nbMoves,
                           bool &searchCompleted,
                           Node *nodeU,
                           Node *nodeV,
                           Node *nodeX,
                           Node *nodeY,
                           Route *routeU,
                           Route *routeV)
{
    SwapStarElement myBestSwapStar;

    if (!bestInsertInitializedForRoute[routeU->cour])
    {
        bestInsertInitializedForRoute[routeU->cour] = true;
        for (int i = 1; i <= params.nbClients; i++)
        {
            bestInsertClient[routeU->cour][i].whenLastCalculated = -1;
            bestInsertClientTW[routeU->cour][i].whenLastCalculated = -1;
        }
    }
    if (!bestInsertInitializedForRoute[routeV->cour])
    {
        bestInsertInitializedForRoute[routeV->cour] = true;
        for (int i = 1; i <= params.nbClients; i++)
        {
            bestInsertClient[routeV->cour][i].whenLastCalculated = -1;
            bestInsertClientTW[routeV->cour][i].whenLastCalculated = -1;
        }
    }

    // Preprocessing insertion costs
    if (withTW)
    {
        preprocessInsertionsWithTW(routeU, routeV, nbMoves);
        preprocessInsertionsWithTW(routeV, routeU, nbMoves);
    }
    else
    {
        preprocessInsertions(routeU, routeV, nbMoves);
        preprocessInsertions(routeV, routeU, nbMoves);
    }

    // Evaluating the moves
    for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
    {
        for (nodeV = routeV->depot->next; !nodeV->isDepot; nodeV = nodeV->next)
        {
            // We cannot determine impact on timewarp without adding too much
            // complexity (O(n^3) instead of O(n^2))
            int const loadPenU = penalties.load(
                routeU->load + params.clients[nodeV->cour].demand
                - params.clients[nodeU->cour].demand);
            int const loadPenV = penalties.load(
                routeV->load + params.clients[nodeU->cour].demand
                - params.clients[nodeV->cour].demand);
            int const deltaLoadPen = loadPenU + loadPenV
                                     - penalties.load(routeU->load)
                                     - penalties.load(routeV->load);
            const int deltaRemoval
                = withTW ? nodeU->deltaRemovalTW + nodeV->deltaRemovalTW
                         : nodeU->deltaRemoval + nodeV->deltaRemoval;

            // Quick filter: possibly early elimination of many SWAP* due to the
            // capacity constraints/penalties and bounds on insertion costs
            if (deltaLoadPen + deltaRemoval <= 0)
            {
                SwapStarElement mySwapStar;
                mySwapStar.U = nodeU;
                mySwapStar.V = nodeV;

                int extraV, extraU;
                if (withTW)
                {
                    // Evaluate best reinsertion cost of U in the route of V
                    // where V has been removed
                    extraV = getCheapestInsertSimultRemovalWithTW(
                        nodeU, nodeV, mySwapStar.bestPositionU);

                    // Evaluate best reinsertion cost of V in the route of U
                    // where U has been removed
                    extraU = getCheapestInsertSimultRemovalWithTW(
                        nodeV, nodeU, mySwapStar.bestPositionV);
                }
                else
                {
                    // Evaluate best reinsertion cost of U in the route of V
                    // where V has been removed
                    extraV = getCheapestInsertSimultRemoval(
                        nodeU, nodeV, mySwapStar.bestPositionU);

                    // Evaluate best reinsertion cost of V in the route of U
                    // where U has been removed
                    extraU = getCheapestInsertSimultRemoval(
                        nodeV, nodeU, mySwapStar.bestPositionV);
                }

                // Evaluating final cost
                mySwapStar.moveCost
                    = deltaLoadPen + deltaRemoval + extraU + extraV;

                if (mySwapStar.moveCost < myBestSwapStar.moveCost)
                {
                    myBestSwapStar = mySwapStar;
                    myBestSwapStar.loadPenU = loadPenU;
                    myBestSwapStar.loadPenV = loadPenV;
                }
            }
        }
    }

    if (!myBestSwapStar.bestPositionU || !myBestSwapStar.bestPositionV)
    {
        return false;
    }

    // Compute actual cost including TimeWarp penalty
    int costSuppU
        = params.dist(myBestSwapStar.bestPositionV->cour,
                      myBestSwapStar.V->cour)
          - params.dist(myBestSwapStar.U->prev->cour, myBestSwapStar.U->cour)
          - params.dist(myBestSwapStar.U->cour, myBestSwapStar.U->next->cour);
    int costSuppV
        = params.dist(myBestSwapStar.bestPositionU->cour,
                      myBestSwapStar.U->cour)
          - params.dist(myBestSwapStar.V->prev->cour, myBestSwapStar.V->cour)
          - params.dist(myBestSwapStar.V->cour, myBestSwapStar.V->next->cour);

    if (myBestSwapStar.bestPositionV == myBestSwapStar.U->prev)
    {
        // Insert in place of U
        costSuppU += params.dist(myBestSwapStar.V->cour,
                                 myBestSwapStar.U->next->cour);
    }
    else
    {
        costSuppU += params.dist(myBestSwapStar.V->cour,
                                 myBestSwapStar.bestPositionV->next->cour)
                     + params.dist(myBestSwapStar.U->prev->cour,
                                   myBestSwapStar.U->next->cour)
                     - params.dist(myBestSwapStar.bestPositionV->cour,
                                   myBestSwapStar.bestPositionV->next->cour);
    }

    if (myBestSwapStar.bestPositionU == myBestSwapStar.V->prev)
    {
        // Insert in place of V
        costSuppV += params.dist(myBestSwapStar.U->cour,
                                 myBestSwapStar.V->next->cour);
    }
    else
    {
        costSuppV += params.dist(myBestSwapStar.U->cour,
                                 myBestSwapStar.bestPositionU->next->cour)
                     + params.dist(myBestSwapStar.V->prev->cour,
                                   myBestSwapStar.V->next->cour)
                     - params.dist(myBestSwapStar.bestPositionU->cour,
                                   myBestSwapStar.bestPositionU->next->cour);
    }

    // It is not possible to have bestPositionU == V or bestPositionV == U, so
    // the positions are always strictly different
    if (myBestSwapStar.bestPositionV->position
        == myBestSwapStar.U->position - 1)
    {
        // Special case
        auto const routeUTwData
            = mergeTwDataRecursive(myBestSwapStar.bestPositionV->prefixTwData,
                                   myBestSwapStar.V->twData,
                                   myBestSwapStar.U->next->postfixTwData);

        costSuppU += penalties.timeWarp(routeUTwData);
    }
    else if (myBestSwapStar.bestPositionV->position
             < myBestSwapStar.U->position)
    {
        auto const routeUTwData = mergeTwDataRecursive(
            myBestSwapStar.bestPositionV->prefixTwData,
            myBestSwapStar.V->twData,
            getRouteSegmentTwData(myBestSwapStar.bestPositionV->next,
                                  myBestSwapStar.U->prev),
            myBestSwapStar.U->next->postfixTwData);

        costSuppU += penalties.timeWarp(routeUTwData);
    }
    else
    {
        auto const routeUTwData = mergeTwDataRecursive(
            myBestSwapStar.U->prev->prefixTwData,
            getRouteSegmentTwData(myBestSwapStar.U->next,
                                  myBestSwapStar.bestPositionV),
            myBestSwapStar.V->twData,
            myBestSwapStar.bestPositionV->next->postfixTwData);

        costSuppU += penalties.timeWarp(routeUTwData);
    }

    if (myBestSwapStar.bestPositionU->position
        == myBestSwapStar.V->position - 1)
    {
        // Special case
        auto const routeVTwData
            = mergeTwDataRecursive(myBestSwapStar.bestPositionU->prefixTwData,
                                   myBestSwapStar.U->twData,
                                   myBestSwapStar.V->next->postfixTwData);

        costSuppV += penalties.timeWarp(routeVTwData);
    }
    else if (myBestSwapStar.bestPositionU->position
             < myBestSwapStar.V->position)
    {
        auto const routeVTwData = mergeTwDataRecursive(
            myBestSwapStar.bestPositionU->prefixTwData,
            myBestSwapStar.U->twData,
            getRouteSegmentTwData(myBestSwapStar.bestPositionU->next,
                                  myBestSwapStar.V->prev),
            myBestSwapStar.V->next->postfixTwData);

        costSuppV += penalties.timeWarp(routeVTwData);
    }
    else
    {
        auto const routeVTwData = mergeTwDataRecursive(
            myBestSwapStar.V->prev->prefixTwData,
            getRouteSegmentTwData(myBestSwapStar.V->next,
                                  myBestSwapStar.bestPositionU),
            myBestSwapStar.U->twData,
            myBestSwapStar.bestPositionU->next->postfixTwData);

        costSuppV += penalties.timeWarp(routeVTwData);
    }

    costSuppU += myBestSwapStar.loadPenU - routeU->penalty;
    costSuppV += myBestSwapStar.loadPenV - routeV->penalty;

    if (costSuppU + costSuppV >= 0)
    {
        return false;
    }

    // Applying the best move in case of improvement
    insertNode(myBestSwapStar.U, myBestSwapStar.bestPositionU);
    insertNode(myBestSwapStar.V, myBestSwapStar.bestPositionV);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU, nbMoves);
    updateRouteData(routeV, nbMoves);

    return true;
}

bool LocalSearch::RelocateStar(int &nbMoves,
                               bool &searchCompleted,
                               Node *nodeU,
                               Node *nodeV,
                               Node *nodeX,
                               Node *nodeY,
                               Route *routeU,
                               Route *routeV)
{
    int bestCost = 0;
    Node *insertionPoint = nullptr;
    Node *nodeToInsert = nullptr;
    for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
    {
        routeU = nodeU->route;
        nodeX = nodeU->next;

        TimeWindowData const routeUTwData = mergeTwDataRecursive(
            nodeU->prev->prefixTwData, nodeX->postfixTwData);
        int const costSuppU
            = params.dist(nodeU->prev->cour, nodeX->cour)
              - params.dist(nodeU->prev->cour, nodeU->cour)
              - params.dist(nodeU->cour, nodeX->cour)
              + penalties.load(routeU->load
                               - params.clients[nodeU->cour].demand)
              + penalties.timeWarp(routeUTwData) - routeU->penalty;

        for (Node *V = routeV->depot->next; !V->isDepot; V = V->next)
        {
            TimeWindowData const routeVTwData = mergeTwDataRecursive(
                V->prefixTwData, nodeU->twData, V->next->postfixTwData);
            int const costSuppV
                = params.dist(V->cour, nodeU->cour)
                  + params.dist(nodeU->cour, V->next->cour)
                  - params.dist(V->cour, V->next->cour)
                  + penalties.load(routeV->load
                                   + params.clients[nodeU->cour].demand)
                  + penalties.timeWarp(routeVTwData) - routeV->penalty;
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

    routeU = nodeToInsert->route;
    insertNode(nodeToInsert, insertionPoint);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU, nbMoves);
    updateRouteData(insertionPoint->route, nbMoves);

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
                    + penalties.timeWarp(twData)
                    - penalties.timeWarp(V->route->twData);

    if (!found || deltaCost < bestCost)
    {
        bestPosition = V->prev;
        bestCost = deltaCost;
    }

    return bestCost;
}

void LocalSearch::preprocessInsertions(Route *R1, Route *R2, int nbMoves)
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

void LocalSearch::preprocessInsertionsWithTW(Route *R1, Route *R2, int nbMoves)
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
                                + penalties.timeWarp(twData)
                                - penalties.timeWarp(R1->twData);
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
                       + penalties.timeWarp(twData)
                       - penalties.timeWarp(R2->twData);

            currentOption.add(cost, R2->depot);

            for (Node *V = R2->depot->next; !V->isDepot; V = V->next)
            {
                twData = mergeTwDataRecursive(
                    V->prefixTwData, U->twData, V->next->postfixTwData);

                int deltaCost = params.dist(V->cour, U->cour)
                                + params.dist(U->cour, V->next->cour)
                                - params.dist(V->cour, V->next->cour)
                                + penalties.timeWarp(twData)
                                - penalties.timeWarp(R2->twData);

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

void LocalSearch::updateRouteData(Route *myRoute, int nbMoves)
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
        = penalties.load(myload) + penalties.timeWarp(myRoute->twData);
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

        updateRouteData(&routes[r], 0);
        routes[r].whenLastTestedLargeNb = -1;
        bestInsertInitializedForRoute[r] = false;
    }

    for (int i = 1; i <= params.nbClients; i++)
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

    std::vector<std::vector<int>> indivRoutes(params.nbVehicles);

    for (int r = 0; r < params.nbVehicles; r++)
    {
        Node *node = depots[routePolarAngles[r].second].next;

        while (!node->isDepot)
        {
            indivRoutes[r].push_back(node->cour);
            node = node->next;
        }
    }

    return {&params, indivRoutes};
}

LocalSearch::LocalSearch(Params &params, XorShift128 &rng)
    : penalties{&params, params.penaltyCapacity, params.penaltyTimeWarp},
      params(params),
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
