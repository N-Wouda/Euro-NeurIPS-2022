#include "LocalSearch.h"

#include "CircleSector.h"
#include "Individual.h"
#include "Params.h"

#include <cmath>
#include <set>
#include <vector>

bool operator==(const TimeWindowData &twData1, const TimeWindowData &twData2)
{
    return twData1.firstNodeIndex == twData2.firstNodeIndex
           && twData1.lastNodeIndex == twData2.lastNodeIndex
           && twData1.duration == twData2.duration
           && twData1.timeWarp == twData2.timeWarp
           && twData1.twEarly == twData2.twEarly
           && twData1.twLate == twData2.twLate;
}

void LocalSearch::initializeConstruction(
    std::vector<NodeToInsert> *nodesToInsert)
{
    // Initialize datastructures relevant for constructions.
    // Local search-related data structures are not initialized.
    emptyRoutes.clear();
    TimeWindowData depotTwData;
    depotTwData.firstNodeIndex = 0;
    depotTwData.lastNodeIndex = 0;
    depotTwData.duration = 0;
    depotTwData.timeWarp = 0;
    depotTwData.twEarly = params.cli[0].twEarly;
    depotTwData.twLate = params.cli[0].twLate;

    // Initializing time window data for clients
    for (int i = 1; i <= params.nbClients; i++)
    {
        TimeWindowData *myTwData = &clients[i].twData;
        myTwData->firstNodeIndex = i;
        myTwData->lastNodeIndex = i;
        myTwData->duration = params.cli[i].servDur;
        myTwData->twEarly = params.cli[i].twEarly;
        myTwData->twLate = params.cli[i].twLate;
    }

    // Initialize routes
    for (int r = 0; r < params.nbVehicles; r++)
    {
        Node *myDepot = &depots[r];
        Node *myDepotFin = &depotsEnd[r];
        myDepot->prev = myDepotFin;
        myDepotFin->next = myDepot;
        myDepot->next = myDepotFin;
        myDepotFin->prev = myDepot;

        myDepot->twData = depotTwData;
        myDepot->prefixTwData = depotTwData;
        myDepot->postfixTwData = depotTwData;

        myDepotFin->twData = depotTwData;
        myDepotFin->prefixTwData = depotTwData;
        myDepotFin->postfixTwData = depotTwData;

        updateRouteData(&routes[r]);
    }

    // Initialize clients.
    for (int i = 1; i <= params.nbClients; i++)
    {
        NodeToInsert nodeToInsert;
        nodeToInsert.clientIdx = i;
        nodeToInsert.twData = clients[i].twData;
        nodeToInsert.load = params.cli[i].demand;
        nodeToInsert.angleFromDepot = atan2(params.cli[i].y - params.cli[0].y,
                                            params.cli[i].x - params.cli[0].x);
        nodeToInsert.serviceDuration = params.cli[i].servDur;
        nodesToInsert->push_back(nodeToInsert);
    }
}

Individual LocalSearch::constructIndividualBySweep(int fillPercentage)
{
    std::vector<NodeToInsert> nodesToInsert;
    initializeConstruction(&nodesToInsert);

    std::vector<std::vector<int>> nodeIndicesPerRoute;

    // Sort nodes according to angle with depot.
    std::sort(std::begin(nodesToInsert),
              std::end(nodesToInsert),
              [](NodeToInsert a, NodeToInsert b) {
                  return a.angleFromDepot < b.angleFromDepot;
              });

    // Distribute clients over routes.
    int load = 0;
    std::vector<int> nodeIndicesInRoute;
    for (int i = 0; i < static_cast<int>(nodesToInsert.size()); i++)
    {
        if (load > 0
            && load + nodesToInsert[i].load
                   > fillPercentage * params.vehicleCapacity / 100
            && nodeIndicesPerRoute.size() + 1 < routes.size())
        {
            nodeIndicesPerRoute.push_back(nodeIndicesInRoute);
            nodeIndicesInRoute.clear();
            load = 0;
        }

        load += nodesToInsert[i].load;
        nodeIndicesInRoute.push_back(i);
    }

    nodeIndicesPerRoute.push_back(nodeIndicesInRoute);

    // Construct routes
    for (int r = 0; r < static_cast<int>(nodeIndicesPerRoute.size()); r++)
    {
        int depotOpeningDuration
            = depots[r].twData.twLate - depots[r].twData.twEarly;
        std::vector<int> nodeIndicesToInsertShortTw;
        std::vector<int> nodeIndicesToInsertLongTw;
        for (int idx : nodeIndicesPerRoute[r])
        {
            // Arbitrary division, but for all instances time windows are either
            // much shorter than half of depotOpeningDuration, or much larger.
            if ((nodesToInsert[idx].twData.twLate
                 - nodesToInsert[idx].twData.twEarly)
                    * 2
                > depotOpeningDuration)
                nodeIndicesToInsertLongTw.push_back(idx);
            else
                nodeIndicesToInsertShortTw.push_back(idx);
        }

        // Sort routes with short time window in increasing end of time window.
        std::sort(std::begin(nodeIndicesToInsertShortTw),
                  std::end(nodeIndicesToInsertShortTw),
                  [&nodesToInsert](int a, int b) {
                      return nodesToInsert[a].twData.twLate
                             < nodesToInsert[b].twData.twLate;
                  });

        // Insert nodes with short time window in order in the route.
        Node *prev = routes[r].depot;
        for (int idx : nodeIndicesToInsertShortTw)
        {
            Node *toInsert = &clients[nodesToInsert[idx].clientIdx];
            Node *insertionPoint = prev;
            toInsert->prev = insertionPoint;
            toInsert->next = insertionPoint->next;
            insertionPoint->next->prev = toInsert;
            insertionPoint->next = toInsert;
            prev = toInsert;
        }

        updateRouteData(&routes[r]);

        // Insert remaining nodes according to best distance
        for (int idx : nodeIndicesToInsertLongTw)
        {
            double bestCost = std::numeric_limits<double>::max();
            Node *bestPred = nullptr;
            Node *prev = routes[r].depot;
            for (int j = 0; j <= routes[r].nbCustomers; j++)
            {
                // Compute insertion cost
                double insertionCost
                    = params.timeCost.get(prev->cour,
                                          nodesToInsert[idx].clientIdx)
                      + params.timeCost.get(nodesToInsert[idx].clientIdx,
                                            prev->next->cour)
                      - params.timeCost.get(prev->cour, prev->next->cour);

                if (insertionCost < bestCost)
                {
                    bestCost = insertionCost;
                    bestPred = prev;
                }

                prev = prev->next;
            }

            Node *toInsert = &clients[nodesToInsert[idx].clientIdx];
            Node *insertionPoint = bestPred;
            toInsert->prev = insertionPoint;
            toInsert->next = insertionPoint->next;
            insertionPoint->next->prev = toInsert;
            insertionPoint->next = toInsert;
            updateRouteData(&routes[r]);
        }
    }

    return exportIndividual();
}

Individual LocalSearch::constructIndividualWithSeedOrder(
    int toleratedCapacityViolation,
    int toleratedTimeWarp,
    bool useSeedClientFurthestFromDepot)
{
    std::vector<NodeToInsert> nodesToInsert;
    initializeConstruction(&nodesToInsert);

    std::set<int> unassignedNodeIndices;
    for (int i = 1; i <= params.nbClients; i++)
    {
        unassignedNodeIndices.insert(i - 1);
    }

    // Construct routes
    for (size_t r = 0; r < routes.size() && !unassignedNodeIndices.empty(); r++)
    {
        // Note that if the seed client is the unassigned client closest to the
        // depot, we do not have to do any initialization and can just start
        // inserting nodes that are best according to distance in the main loop.
        if (useSeedClientFurthestFromDepot)
        {
            int furthestNodeIdx = -1;
            double furthestNodeCost = -1.0;
            for (int idx : unassignedNodeIndices)
            {
                double insertionCost
                    = params.timeCost.get(routes[r].depot->cour,
                                          nodesToInsert[idx].clientIdx)
                      + params.timeCost.get(nodesToInsert[idx].clientIdx,
                                            routes[r].depot->next->cour)
                      - params.timeCost.get(routes[r].depot->cour,
                                            routes[r].depot->next->cour);

                if (insertionCost > furthestNodeCost)
                {
                    furthestNodeCost = insertionCost;
                    furthestNodeIdx = idx;
                }
            }

            Node *toInsert = &clients[nodesToInsert[furthestNodeIdx].clientIdx];
            toInsert->prev = routes[r].depot;
            toInsert->next = routes[r].depot->next;
            routes[r].depot->next->prev = toInsert;
            routes[r].depot->next = toInsert;
            updateRouteData(&routes[r]);
            unassignedNodeIndices.erase(furthestNodeIdx);
        }

        bool insertedNode = true;
        while (insertedNode)
        {
            insertedNode = false;
            double bestCost = std::numeric_limits<double>::max();
            Node *bestPred = nullptr;
            int bestNodeIdx;
            for (int idx : unassignedNodeIndices)
            {
                // Do not allow insertion if capacity is exceeded more than
                // tolerance.
                if (routes[r].load + nodesToInsert[idx].load
                    > params.vehicleCapacity + toleratedCapacityViolation)
                    continue;

                Node *prev = routes[r].depot;
                for (int j = 0; j <= routes[r].nbCustomers; j++)
                {
                    // Do not allow insertions if time windows are violated more
                    // than tolerance
                    TimeWindowData routeTwData
                        = MergeTWDataRecursive(prev->prefixTwData,
                                               nodesToInsert[idx].twData,
                                               prev->next->postfixTwData);
                    if (routeTwData.timeWarp > toleratedTimeWarp)
                    {
                        prev = prev->next;
                        continue;
                    }

                    // Compute insertion cost
                    double insertionCost
                        = params.timeCost.get(prev->cour,
                                              nodesToInsert[idx].clientIdx)
                          + params.timeCost.get(nodesToInsert[idx].clientIdx,
                                                prev->next->cour)
                          - params.timeCost.get(prev->cour, prev->next->cour);

                    if (insertionCost < bestCost)
                    {
                        bestCost = insertionCost;
                        bestPred = prev;
                        bestNodeIdx = idx;
                    }

                    prev = prev->next;
                }
            }

            if (bestCost < std::numeric_limits<double>::max())
            {
                Node *toInsert = &clients[nodesToInsert[bestNodeIdx].clientIdx];
                toInsert->prev = bestPred;
                toInsert->next = bestPred->next;
                bestPred->next->prev = toInsert;
                bestPred->next = toInsert;
                updateRouteData(&routes[r]);
                insertedNode = true;
                unassignedNodeIndices.erase(bestNodeIdx);
            }
        }
    }

    // Insert all unassigned nodes at the back of the last route. We assume that
    // typically there are no unassigned nodes left, because there are plenty
    // routes, but we have to make sure that all nodes are assigned.
    if (!unassignedNodeIndices.empty())
    {
        size_t lastRouteIdx = routes.size() - 1;
        Node *prevNode = depotsEnd[lastRouteIdx].prev;  // node before depot

        while (!unassignedNodeIndices.empty())
        {
            int idx = *unassignedNodeIndices.begin();
            Node *toInsert = &clients[nodesToInsert[idx].clientIdx];
            toInsert->prev = prevNode;
            toInsert->next = prevNode->next;
            prevNode->next->prev = toInsert;
            prevNode->next = toInsert;
            unassignedNodeIndices.erase(idx);
        }

        updateRouteData(&routes[lastRouteIdx]);
    }

    return exportIndividual();
}

void LocalSearch::run(Individual *indiv,
                      double penaltyCapacityLS,
                      double penaltyTimeWarpLS)
{
    static const bool neverIntensify
        = params.config.intensificationProbabilityLS == 0;
    static const bool alwaysIntensify
        = params.config.intensificationProbabilityLS == 100;
    const bool runLS_INT
        = rng.randint(100)
          < (unsigned int)params.config.intensificationProbabilityLS;

    this->penaltyCapacityLS = penaltyCapacityLS;
    this->penaltyTimeWarpLS = penaltyTimeWarpLS;
    loadIndividual(indiv);

    // Shuffling the order of the nodes explored by the LS to allow for more
    // diversity in the search
    std::shuffle(orderNodes.begin(), orderNodes.end(), rng);
    std::shuffle(orderRoutes.begin(), orderRoutes.end(), rng);
    for (int i = 1; i <= params.nbClients; i++)
        if (rng.randint(params.config.nbGranular)
            == 0)  // Designed to use O(nbGranular x n) time overall to avoid
                   // possible bottlenecks
            std::shuffle(params.correlatedVertices[i].begin(),
                         params.correlatedVertices[i].end(),
                         rng);

    searchCompleted = false;
    for (loopID = 0; !searchCompleted; loopID++)
    {
        if (loopID > 1)
        {
            // Allows at least two loops since some moves involving empty routes
            // are not checked at the first loop
            searchCompleted = true;
        }

        /* CLASSICAL ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY
         * RESTRICTION */
        for (int posU = 0; posU < params.nbClients; posU++)
        {
            nodeU = &clients[orderNodes[posU]];
            int lastTestRINodeU = nodeU->whenLastTestedRI;
            nodeU->whenLastTestedRI = nbMoves;

            const auto &correlated = params.correlatedVertices[nodeU->cour];

            for (const auto &v : correlated)
            {
                nodeV = &clients[v];
                if (loopID == 0
                    || std::max(nodeU->route->whenLastModified,
                                nodeV->route->whenLastModified)
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
                    setLocalVariablesRouteU();
                    setLocalVariablesRouteV();
                    if (MoveSingleClient())
                        continue;  // RELOCATE
                    if (MoveTwoClients())
                        continue;  // RELOCATE
                    if (MoveTwoClientsReversed())
                        continue;  // RELOCATE
                    if (nodeUIndex < nodeVIndex && SwapTwoSingleClients())
                        continue;  // SWAP
                    if (SwapTwoClientsForOne())
                        continue;  // SWAP
                    if (nodeUIndex < nodeVIndex && SwapTwoClientPairs())
                        continue;  // SWAP
                    if (routeU->cour < routeV->cour && TwoOptBetweenTrips())
                        continue;  // 2-OPT*
                    if (routeU == routeV && TwoOptWithinTrip())
                        continue;  // 2-OPT

                    // Trying moves that insert nodeU directly after the depot
                    if (nodeV->prev->isDepot)
                    {
                        nodeV = nodeV->prev;
                        setLocalVariablesRouteV();
                        if (MoveSingleClient())
                            continue;  // RELOCATE
                        if (MoveTwoClients())
                            continue;  // RELOCATE
                        if (MoveTwoClientsReversed())
                            continue;  // RELOCATE
                        if (routeU->cour < routeV->cour && TwoOptBetweenTrips())
                            continue;  // 2-OPT*
                    }
                }
            }

            /* MOVES INVOLVING AN EMPTY ROUTE -- NOT TESTED IN THE FIRST LOOP TO
             * AVOID INCREASING TOO MUCH THE FLEET SIZE */
            if (loopID > 0 && !emptyRoutes.empty())
            {
                nodeV = routes[*emptyRoutes.begin()].depot;
                setLocalVariablesRouteU();
                setLocalVariablesRouteV();
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
        if (!neverIntensify && searchCompleted
            && (alwaysIntensify || runLS_INT))
        {
            for (int rU = 0; rU < params.nbVehicles; rU++)
            {
                routeU = &routes[orderRoutes[rU]];
                if (routeU->nbCustomers == 0)
                {
                    continue;
                }

                int lastTestLargeNbRouteU = routeU->whenLastTestedLargeNb;
                routeU->whenLastTestedLargeNb = nbMoves;
                for (int rV = 0; rV < params.nbVehicles; rV++)
                {
                    routeV = &routes[orderRoutes[rV]];
                    if (routeV->nbCustomers == 0
                        || routeU->cour >= routeV->cour)
                    {
                        continue;
                    }

                    if (loopID > 0
                        && std::max(routeU->whenLastModified,
                                    routeV->whenLastModified)
                               <= lastTestLargeNbRouteU)
                    {
                        continue;
                    }

                    if (!CircleSector::overlap(
                            routeU->sector,
                            routeV->sector,
                            params.config.circleSectorOverlapTolerance))
                    {
                        continue;
                    }

                    if (!RelocateStar())
                    {
                        if (params.config.skipSwapStarDist || !swapStar(false))
                        {
                            if (params.config.useSwapStarTW)
                            {
                                swapStar(true);
                            }
                        }
                    }
                }
            }
        }
    }

    // Register the solution produced by the LS in the individual
    *indiv = exportIndividual();
}

void LocalSearch::setLocalVariablesRouteU()
{
    routeU = nodeU->route;
    nodeX = nodeU->next;
    nodeXNextIndex = nodeX->next->cour;
    nodeUIndex = nodeU->cour;
    nodeUPrevIndex = nodeU->prev->cour;
    nodeXIndex = nodeX->cour;
    loadU = params.cli[nodeUIndex].demand;
    loadX = params.cli[nodeXIndex].demand;
    routeUTimeWarp = routeU->twData.timeWarp > 0;
    routeULoadPenalty = routeU->load > params.vehicleCapacity;
}

void LocalSearch::setLocalVariablesRouteV()
{
    routeV = nodeV->route;
    nodeY = nodeV->next;
    nodeYNextIndex = nodeY->next->cour;
    nodeVIndex = nodeV->cour;
    nodeVPrevIndex = nodeV->prev->cour;
    nodeYIndex = nodeY->cour;
    loadV = params.cli[nodeVIndex].demand;
    loadY = params.cli[nodeYIndex].demand;
    routeVTimeWarp = routeV->twData.timeWarp > 0;
    routeVLoadPenalty = routeV->load > params.vehicleCapacity;
}

bool LocalSearch::MoveSingleClient()
{
    // If U already comes directly after V, this move has no effect
    if (nodeUIndex == nodeYIndex)
        return false;

    double costSuppU = params.timeCost.get(nodeUPrevIndex, nodeXIndex)
                       - params.timeCost.get(nodeUPrevIndex, nodeUIndex)
                       - params.timeCost.get(nodeUIndex, nodeXIndex);
    double costSuppV = params.timeCost.get(nodeVIndex, nodeUIndex)
                       + params.timeCost.get(nodeUIndex, nodeYIndex)
                       - params.timeCost.get(nodeVIndex, nodeYIndex);
    TimeWindowData routeUTwData, routeVTwData;

    if (routeU != routeV)
    {
        if (!routeULoadPenalty && !routeUTimeWarp
            && costSuppU + costSuppV > -MY_EPSILON)
        {
            return false;
        }

        routeUTwData = MergeTWDataRecursive(nodeU->prev->prefixTwData,
                                            nodeX->postfixTwData);
        routeVTwData = MergeTWDataRecursive(
            nodeV->prefixTwData, nodeU->twData, nodeY->postfixTwData);

        costSuppU += penaltyExcessLoad(routeU->load - loadU)
                     + penaltyTimeWindows(routeUTwData) - routeU->penalty;

        costSuppV += penaltyExcessLoad(routeV->load + loadU)
                     + penaltyTimeWindows(routeVTwData) - routeV->penalty;
    }
    else
    {
        if (!routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
        {
            return false;
        }

        // Move within the same route
        if (nodeU->position < nodeV->position)
        {
            // Edge case V directly after U, so X == V, this works
            // start - ... - UPrev - X - ... - V - U - Y - ... - end
            routeUTwData
                = MergeTWDataRecursive(nodeU->prev->prefixTwData,
                                       getRouteSegmentTwData(nodeX, nodeV),
                                       nodeU->twData,
                                       nodeY->postfixTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - U - Y - ... - UPrev - X - ... - end
            routeUTwData = MergeTWDataRecursive(
                nodeV->prefixTwData,
                nodeU->twData,
                getRouteSegmentTwData(nodeY, nodeU->prev),
                nodeX->postfixTwData);
        }

        // Compute new total penalty
        costSuppU += penaltyExcessLoad(routeU->load)
                     + penaltyTimeWindows(routeUTwData) - routeU->penalty;
    }

    if (costSuppU + costSuppV > -MY_EPSILON)
        return false;

    insertNode(nodeU, nodeV);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    if (routeU != routeV)
        updateRouteData(routeV);

    return true;
}

bool LocalSearch::MoveTwoClients()
{
    if (nodeU == nodeY || nodeV == nodeX || nodeX->isDepot)
        return false;

    double costSuppU = params.timeCost.get(nodeUPrevIndex, nodeXNextIndex)
                       - params.timeCost.get(nodeUPrevIndex, nodeUIndex)
                       - params.timeCost.get(nodeXIndex, nodeXNextIndex);
    double costSuppV = params.timeCost.get(nodeVIndex, nodeUIndex)
                       + params.timeCost.get(nodeXIndex, nodeYIndex)
                       - params.timeCost.get(nodeVIndex, nodeYIndex);
    TimeWindowData routeUTwData, routeVTwData;

    if (routeU != routeV)
    {
        if (!routeULoadPenalty && !routeUTimeWarp
            && costSuppU + costSuppV > -MY_EPSILON)
        {
            return false;
        }

        routeUTwData = MergeTWDataRecursive(nodeU->prev->prefixTwData,
                                            nodeX->next->postfixTwData);
        routeVTwData = MergeTWDataRecursive(nodeV->prefixTwData,
                                            getEdgeTwData(nodeU, nodeX),
                                            nodeY->postfixTwData);

        costSuppU += penaltyExcessLoad(routeU->load - loadU - loadX)
                     + penaltyTimeWindows(routeUTwData) - routeU->penalty;

        costSuppV += penaltyExcessLoad(routeV->load + loadU + loadX)
                     + penaltyTimeWindows(routeVTwData) - routeV->penalty;
    }
    else
    {
        if (!routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
        {
            return false;
        }

        // Move within the same route
        if (nodeU->position < nodeV->position)
        {
            // Edge case V directly after U, so X == V is excluded, V directly
            // after X so XNext == V works start - ... - UPrev - XNext - ... - V
            // - U - X - Y - ... - end
            routeUTwData = MergeTWDataRecursive(
                nodeU->prev->prefixTwData,
                getRouteSegmentTwData(nodeX->next, nodeV),
                getEdgeTwData(nodeU, nodeX),
                nodeY->postfixTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - U - X - Y - ... - UPrev - XNext - ...
            // - end
            routeUTwData = MergeTWDataRecursive(
                nodeV->prefixTwData,
                getEdgeTwData(nodeU, nodeX),
                getRouteSegmentTwData(nodeY, nodeU->prev),
                nodeX->next->postfixTwData);
        }

        // Compute new total penalty
        costSuppU += penaltyExcessLoad(routeU->load)
                     + penaltyTimeWindows(routeUTwData) - routeU->penalty;
    }

    if (costSuppU + costSuppV > -MY_EPSILON)
        return false;

    insertNode(nodeU, nodeV);
    insertNode(nodeX, nodeU);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    if (routeU != routeV)
        updateRouteData(routeV);

    return true;
}

bool LocalSearch::MoveTwoClientsReversed()
{
    if (nodeU == nodeY || nodeX == nodeV || nodeX->isDepot)
        return false;

    double costSuppU = params.timeCost.get(nodeUPrevIndex, nodeXNextIndex)
                       - params.timeCost.get(nodeUPrevIndex, nodeUIndex)
                       - params.timeCost.get(nodeUIndex, nodeXIndex)
                       - params.timeCost.get(nodeXIndex, nodeXNextIndex);
    double costSuppV = params.timeCost.get(nodeVIndex, nodeXIndex)
                       + params.timeCost.get(nodeXIndex, nodeUIndex)
                       + params.timeCost.get(nodeUIndex, nodeYIndex)
                       - params.timeCost.get(nodeVIndex, nodeYIndex);
    TimeWindowData routeUTwData, routeVTwData;

    if (routeU != routeV)
    {
        if (!routeULoadPenalty && !routeUTimeWarp
            && costSuppU + costSuppV > -MY_EPSILON)
        {
            return false;
        }

        routeUTwData = MergeTWDataRecursive(nodeU->prev->prefixTwData,
                                            nodeX->next->postfixTwData);
        routeVTwData = MergeTWDataRecursive(nodeV->prefixTwData,
                                            getEdgeTwData(nodeX, nodeU),
                                            nodeY->postfixTwData);

        costSuppU += penaltyExcessLoad(routeU->load - loadU - loadX)
                     + penaltyTimeWindows(routeUTwData) - routeU->penalty;

        costSuppV += penaltyExcessLoad(routeV->load + loadU + loadX)
                     + penaltyTimeWindows(routeVTwData) - routeV->penalty;
    }
    else
    {
        if (!routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
        {
            return false;
        }

        // Move within the same route
        if (nodeU->position < nodeV->position)
        {
            // Edge case V directly after U, so X == V is excluded, V directly
            // after X so XNext == V works start - ... - UPrev - XNext - ... - V
            // - X - U - Y - ... - end
            routeUTwData = MergeTWDataRecursive(
                nodeU->prev->prefixTwData,
                getRouteSegmentTwData(nodeX->next, nodeV),
                getEdgeTwData(nodeX, nodeU),
                nodeY->postfixTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - X - U - Y - ... - UPrev - XNext - ...
            // - end
            routeUTwData = MergeTWDataRecursive(
                nodeV->prefixTwData,
                getEdgeTwData(nodeX, nodeU),
                getRouteSegmentTwData(nodeY, nodeU->prev),
                nodeX->next->postfixTwData);
        }

        // Compute new total penalty
        costSuppU += penaltyExcessLoad(routeU->load)
                     + penaltyTimeWindows(routeUTwData) - routeU->penalty;
    }

    if (costSuppU + costSuppV > -MY_EPSILON)
        return false;

    insertNode(nodeX, nodeV);
    insertNode(nodeU, nodeX);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    if (routeU != routeV)
        updateRouteData(routeV);

    return true;
}

bool LocalSearch::SwapTwoSingleClients()
{
    if (nodeUIndex == nodeVPrevIndex || nodeUIndex == nodeYIndex)
        return false;

    double costSuppU = params.timeCost.get(nodeUPrevIndex, nodeVIndex)
                       + params.timeCost.get(nodeVIndex, nodeXIndex)
                       - params.timeCost.get(nodeUPrevIndex, nodeUIndex)
                       - params.timeCost.get(nodeUIndex, nodeXIndex);
    double costSuppV = params.timeCost.get(nodeVPrevIndex, nodeUIndex)
                       + params.timeCost.get(nodeUIndex, nodeYIndex)
                       - params.timeCost.get(nodeVPrevIndex, nodeVIndex)
                       - params.timeCost.get(nodeVIndex, nodeYIndex);
    TimeWindowData routeUTwData, routeVTwData;

    if (routeU != routeV)
    {
        if (!routeULoadPenalty && !routeUTimeWarp && !routeVLoadPenalty
            && !routeVTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
        {
            return false;
        }

        routeUTwData = MergeTWDataRecursive(
            nodeU->prev->prefixTwData, nodeV->twData, nodeX->postfixTwData);
        routeVTwData = MergeTWDataRecursive(
            nodeV->prev->prefixTwData, nodeU->twData, nodeY->postfixTwData);

        costSuppU += penaltyExcessLoad(routeU->load + loadV - loadU)
                     + penaltyTimeWindows(routeUTwData) - routeU->penalty;

        costSuppV += penaltyExcessLoad(routeV->load + loadU - loadV)
                     + penaltyTimeWindows(routeVTwData) - routeV->penalty;
    }
    else
    {
        if (!routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
        {
            return false;
        }

        // Swap within the same route
        if (nodeU->position < nodeV->position)
        {
            // Edge case V directly after U, so X == V is excluded, V directly
            // after X so XNext == V works start - ... - UPrev - V - X - ... -
            // VPrev - U - Y - ... - end
            routeUTwData = MergeTWDataRecursive(
                nodeU->prev->prefixTwData,
                nodeV->twData,
                getRouteSegmentTwData(nodeX, nodeV->prev),
                nodeU->twData,
                nodeY->postfixTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - VPrev - U - Y - ... - UPrev - V - X - ...
            // - end
            routeUTwData = MergeTWDataRecursive(
                nodeV->prev->prefixTwData,
                nodeU->twData,
                getRouteSegmentTwData(nodeY, nodeU->prev),
                nodeV->twData,
                nodeX->postfixTwData);
        }

        // Compute new total penalty
        costSuppU += penaltyExcessLoad(routeU->load)
                     + penaltyTimeWindows(routeUTwData) - routeU->penalty;
    }

    if (costSuppU + costSuppV > -MY_EPSILON)
        return false;

    swapNode(nodeU, nodeV);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    if (routeU != routeV)
        updateRouteData(routeV);

    return true;
}

bool LocalSearch::SwapTwoClientsForOne()
{
    if (nodeU == nodeV->prev || nodeX == nodeV->prev || nodeU == nodeY
        || nodeX->isDepot)
        return false;

    double costSuppU = params.timeCost.get(nodeUPrevIndex, nodeVIndex)
                       + params.timeCost.get(nodeVIndex, nodeXNextIndex)
                       - params.timeCost.get(nodeUPrevIndex, nodeUIndex)
                       - params.timeCost.get(nodeXIndex, nodeXNextIndex);
    double costSuppV = params.timeCost.get(nodeVPrevIndex, nodeUIndex)
                       + params.timeCost.get(nodeXIndex, nodeYIndex)
                       - params.timeCost.get(nodeVPrevIndex, nodeVIndex)
                       - params.timeCost.get(nodeVIndex, nodeYIndex);
    TimeWindowData routeUTwData, routeVTwData;

    if (routeU != routeV)
    {
        if (!routeULoadPenalty && !routeUTimeWarp && !routeVLoadPenalty
            && !routeVTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
        {
            return false;
        }

        routeUTwData = MergeTWDataRecursive(nodeU->prev->prefixTwData,
                                            nodeV->twData,
                                            nodeX->next->postfixTwData);
        routeVTwData = MergeTWDataRecursive(nodeV->prev->prefixTwData,
                                            getEdgeTwData(nodeU, nodeX),
                                            nodeY->postfixTwData);

        costSuppU += penaltyExcessLoad(routeU->load + loadV - loadU - loadX)
                     + penaltyTimeWindows(routeUTwData) - routeU->penalty;

        costSuppV += penaltyExcessLoad(routeV->load + loadU + loadX - loadV)
                     + penaltyTimeWindows(routeVTwData) - routeV->penalty;
    }
    else
    {
        if (!routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
        {
            return false;
        }

        // Swap within the same route
        if (nodeU->position < nodeV->position)
        {
            // start - ... - UPrev - V - XNext - ... - VPrev - U - X - Y - ... -
            // end
            routeUTwData = MergeTWDataRecursive(
                nodeU->prev->prefixTwData,
                nodeV->twData,
                getRouteSegmentTwData(nodeX->next, nodeV->prev),
                getEdgeTwData(nodeU, nodeX),
                nodeY->postfixTwData);
        }
        else
        {
            // start - ... - VPrev - U - X - Y - ... - UPrev - V - XNext - ... -
            // end
            routeUTwData = MergeTWDataRecursive(
                nodeV->prev->prefixTwData,
                getEdgeTwData(nodeU, nodeX),
                getRouteSegmentTwData(nodeY, nodeU->prev),
                nodeV->twData,
                nodeX->next->postfixTwData);
        }

        // Compute new total penalty
        costSuppU += penaltyExcessLoad(routeU->load)
                     + penaltyTimeWindows(routeUTwData) - routeU->penalty;
    }

    if (costSuppU + costSuppV > -MY_EPSILON)
        return false;

    // Note: next two lines are a bit inefficient but we only update
    // occasionally and updateRouteData is much more costly anyway, efficient
    // checks are more important
    swapNode(nodeU, nodeV);
    insertNode(nodeX, nodeU);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    if (routeU != routeV)
        updateRouteData(routeV);

    return true;
}

bool LocalSearch::SwapTwoClientPairs()
{
    if (nodeX->isDepot || nodeY->isDepot || nodeY == nodeU->prev
        || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next)
        return false;

    double costSuppU = params.timeCost.get(nodeUPrevIndex, nodeVIndex)
                       + params.timeCost.get(nodeYIndex, nodeXNextIndex)
                       - params.timeCost.get(nodeUPrevIndex, nodeUIndex)
                       - params.timeCost.get(nodeXIndex, nodeXNextIndex);
    double costSuppV = params.timeCost.get(nodeVPrevIndex, nodeUIndex)
                       + params.timeCost.get(nodeXIndex, nodeYNextIndex)
                       - params.timeCost.get(nodeVPrevIndex, nodeVIndex)
                       - params.timeCost.get(nodeYIndex, nodeYNextIndex);
    TimeWindowData routeUTwData, routeVTwData;

    if (routeU != routeV)
    {
        if (!routeULoadPenalty && !routeUTimeWarp && !routeVLoadPenalty
            && !routeVTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
        {
            return false;
        }

        routeUTwData = MergeTWDataRecursive(nodeU->prev->prefixTwData,
                                            getEdgeTwData(nodeV, nodeY),
                                            nodeX->next->postfixTwData);
        routeVTwData = MergeTWDataRecursive(nodeV->prev->prefixTwData,
                                            getEdgeTwData(nodeU, nodeX),
                                            nodeY->next->postfixTwData);

        costSuppU
            += penaltyExcessLoad(routeU->load + loadV + loadY - loadU - loadX)
               + penaltyTimeWindows(routeUTwData) - routeU->penalty;

        costSuppV
            += penaltyExcessLoad(routeV->load + loadU + loadX - loadV - loadY)
               + penaltyTimeWindows(routeVTwData) - routeV->penalty;
    }
    else
    {
        if (!routeUTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
        {
            return false;
        }

        // Swap within the same route
        if (nodeU->position < nodeV->position)
        {
            // start - ... - UPrev - V - Y - XNext - ... - VPrev - U - X - YNext
            // - ... - end
            routeUTwData = MergeTWDataRecursive(
                nodeU->prev->prefixTwData,
                getEdgeTwData(nodeV, nodeY),
                getRouteSegmentTwData(nodeX->next, nodeV->prev),
                getEdgeTwData(nodeU, nodeX),
                nodeY->next->postfixTwData);
        }
        else
        {
            // start - ... - VPrev - U - X - YNext - ... - UPrev - V - Y - XNext
            // - ... - end
            routeUTwData = MergeTWDataRecursive(
                nodeV->prev->prefixTwData,
                getEdgeTwData(nodeU, nodeX),
                getRouteSegmentTwData(nodeY->next, nodeU->prev),
                getEdgeTwData(nodeV, nodeY),
                nodeX->next->postfixTwData);
        }

        // Compute new total penalty
        costSuppU += penaltyExcessLoad(routeU->load)
                     + penaltyTimeWindows(routeUTwData) - routeU->penalty;
    }

    if (costSuppU + costSuppV > -MY_EPSILON)
        return false;

    swapNode(nodeU, nodeV);
    swapNode(nodeX, nodeY);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    if (routeU != routeV)
        updateRouteData(routeV);

    return true;
}

bool LocalSearch::TwoOptWithinTrip()
{
    if (nodeU->position >= nodeV->position - 1)
        return false;

    double cost = params.timeCost.get(nodeUIndex, nodeVIndex)
                  + params.timeCost.get(nodeXIndex, nodeYIndex)
                  - params.timeCost.get(nodeUIndex, nodeXIndex)
                  - params.timeCost.get(nodeVIndex, nodeYIndex)
                  + nodeV->cumulatedReversalDistance
                  - nodeX->cumulatedReversalDistance;

    if (!routeUTimeWarp && cost > -MY_EPSILON)
    {
        return false;
    }

    TimeWindowData routeTwData = nodeU->prefixTwData;
    Node *itRoute = nodeV;
    while (itRoute != nodeU)
    {
        routeTwData = MergeTWDataRecursive(routeTwData, itRoute->twData);
        itRoute = itRoute->prev;
    }
    routeTwData = MergeTWDataRecursive(routeTwData, nodeY->postfixTwData);

    // Compute new total penalty
    cost += penaltyExcessLoad(routeU->load) + penaltyTimeWindows(routeTwData)
            - routeU->penalty;

    if (cost > -MY_EPSILON)
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
    updateRouteData(routeU);

    return true;
}

bool LocalSearch::TwoOptBetweenTrips()
{
    double costSuppU = params.timeCost.get(nodeUIndex, nodeYIndex)
                       - params.timeCost.get(nodeUIndex, nodeXIndex);
    double costSuppV = params.timeCost.get(nodeVIndex, nodeXIndex)
                       - params.timeCost.get(nodeVIndex, nodeYIndex);

    if (!routeULoadPenalty && !routeUTimeWarp && !routeVLoadPenalty
        && !routeVTimeWarp && costSuppU + costSuppV > -MY_EPSILON)
    {
        return false;
    }

    TimeWindowData routeUTwData, routeVTwData;

    routeUTwData
        = MergeTWDataRecursive(nodeU->prefixTwData, nodeY->postfixTwData);
    routeVTwData
        = MergeTWDataRecursive(nodeV->prefixTwData, nodeX->postfixTwData);

    costSuppU += penaltyExcessLoad(nodeU->cumulatedLoad + routeV->load
                                   - nodeV->cumulatedLoad)
                 + penaltyTimeWindows(routeUTwData) - routeU->penalty;

    costSuppV += penaltyExcessLoad(nodeV->cumulatedLoad + routeU->load
                                   - nodeU->cumulatedLoad)
                 + penaltyTimeWindows(routeVTwData) - routeV->penalty;

    if (costSuppU + costSuppV > -MY_EPSILON)
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
    updateRouteData(routeU);
    updateRouteData(routeV);

    return true;
}

bool LocalSearch::swapStar(const bool withTW)
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
        preprocessInsertionsWithTW(routeU, routeV);
        preprocessInsertionsWithTW(routeV, routeU);
    }
    else
    {
        preprocessInsertions(routeU, routeV);
        preprocessInsertions(routeV, routeU);
    }

    // Evaluating the moves
    for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
    {
        for (nodeV = routeV->depot->next; !nodeV->isDepot; nodeV = nodeV->next)
        {
            // We cannot determine impact on timewarp without adding too much
            // complexity (O(n^3) instead of O(n^2))
            const double loadPenU = penaltyExcessLoad(
                routeU->load + params.cli[nodeV->cour].demand
                - params.cli[nodeU->cour].demand);
            const double loadPenV = penaltyExcessLoad(
                routeV->load + params.cli[nodeU->cour].demand
                - params.cli[nodeV->cour].demand);
            const double deltaLoadPen = loadPenU + loadPenV
                                        - penaltyExcessLoad(routeU->load)
                                        - penaltyExcessLoad(routeV->load);
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
    double costSuppU = params.timeCost.get(myBestSwapStar.bestPositionV->cour,
                                           myBestSwapStar.V->cour)
                       - params.timeCost.get(myBestSwapStar.U->prev->cour,
                                             myBestSwapStar.U->cour)
                       - params.timeCost.get(myBestSwapStar.U->cour,
                                             myBestSwapStar.U->next->cour);
    double costSuppV = params.timeCost.get(myBestSwapStar.bestPositionU->cour,
                                           myBestSwapStar.U->cour)
                       - params.timeCost.get(myBestSwapStar.V->prev->cour,
                                             myBestSwapStar.V->cour)
                       - params.timeCost.get(myBestSwapStar.V->cour,
                                             myBestSwapStar.V->next->cour);

    if (myBestSwapStar.bestPositionV == myBestSwapStar.U->prev)
    {
        // Insert in place of U
        costSuppU += params.timeCost.get(myBestSwapStar.V->cour,
                                         myBestSwapStar.U->next->cour);
    }
    else
    {
        costSuppU
            += params.timeCost.get(myBestSwapStar.V->cour,
                                   myBestSwapStar.bestPositionV->next->cour)
               + params.timeCost.get(myBestSwapStar.U->prev->cour,
                                     myBestSwapStar.U->next->cour)
               - params.timeCost.get(myBestSwapStar.bestPositionV->cour,
                                     myBestSwapStar.bestPositionV->next->cour);
    }

    if (myBestSwapStar.bestPositionU == myBestSwapStar.V->prev)
    {
        // Insert in place of V
        costSuppV += params.timeCost.get(myBestSwapStar.U->cour,
                                         myBestSwapStar.V->next->cour);
    }
    else
    {
        costSuppV
            += params.timeCost.get(myBestSwapStar.U->cour,
                                   myBestSwapStar.bestPositionU->next->cour)
               + params.timeCost.get(myBestSwapStar.V->prev->cour,
                                     myBestSwapStar.V->next->cour)
               - params.timeCost.get(myBestSwapStar.bestPositionU->cour,
                                     myBestSwapStar.bestPositionU->next->cour);
    }

    TimeWindowData routeUTwData, routeVTwData;
    // It is not possible to have bestPositionU == V or bestPositionV == U, so
    // the positions are always strictly different
    if (myBestSwapStar.bestPositionV->position
        == myBestSwapStar.U->position - 1)
    {
        // Special case
        routeUTwData
            = MergeTWDataRecursive(myBestSwapStar.bestPositionV->prefixTwData,
                                   myBestSwapStar.V->twData,
                                   myBestSwapStar.U->next->postfixTwData);
    }
    else if (myBestSwapStar.bestPositionV->position
             < myBestSwapStar.U->position)
    {
        routeUTwData = MergeTWDataRecursive(
            myBestSwapStar.bestPositionV->prefixTwData,
            myBestSwapStar.V->twData,
            getRouteSegmentTwData(myBestSwapStar.bestPositionV->next,
                                  myBestSwapStar.U->prev),
            myBestSwapStar.U->next->postfixTwData);
    }
    else
    {
        routeUTwData = MergeTWDataRecursive(
            myBestSwapStar.U->prev->prefixTwData,
            getRouteSegmentTwData(myBestSwapStar.U->next,
                                  myBestSwapStar.bestPositionV),
            myBestSwapStar.V->twData,
            myBestSwapStar.bestPositionV->next->postfixTwData);
    }

    if (myBestSwapStar.bestPositionU->position
        == myBestSwapStar.V->position - 1)
    {
        // Special case
        routeVTwData
            = MergeTWDataRecursive(myBestSwapStar.bestPositionU->prefixTwData,
                                   myBestSwapStar.U->twData,
                                   myBestSwapStar.V->next->postfixTwData);
    }
    else if (myBestSwapStar.bestPositionU->position
             < myBestSwapStar.V->position)
    {
        routeVTwData = MergeTWDataRecursive(
            myBestSwapStar.bestPositionU->prefixTwData,
            myBestSwapStar.U->twData,
            getRouteSegmentTwData(myBestSwapStar.bestPositionU->next,
                                  myBestSwapStar.V->prev),
            myBestSwapStar.V->next->postfixTwData);
    }
    else
    {
        routeVTwData = MergeTWDataRecursive(
            myBestSwapStar.V->prev->prefixTwData,
            getRouteSegmentTwData(myBestSwapStar.V->next,
                                  myBestSwapStar.bestPositionU),
            myBestSwapStar.U->twData,
            myBestSwapStar.bestPositionU->next->postfixTwData);
    }

    costSuppU += myBestSwapStar.loadPenU + penaltyTimeWindows(routeUTwData)
                 - routeU->penalty;

    costSuppV += myBestSwapStar.loadPenV + penaltyTimeWindows(routeVTwData)
                 - routeV->penalty;

    if (costSuppU + costSuppV > -MY_EPSILON)
    {
        return false;
    }

    // Applying the best move in case of improvement
    insertNode(myBestSwapStar.U, myBestSwapStar.bestPositionU);
    insertNode(myBestSwapStar.V, myBestSwapStar.bestPositionV);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    updateRouteData(routeU);
    updateRouteData(routeV);

    return true;
}

bool LocalSearch::RelocateStar()
{
    double bestCost = 0;
    Node *insertionPoint = nullptr;
    Node *nodeToInsert = nullptr;
    for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
    {
        setLocalVariablesRouteU();

        const TimeWindowData routeUTwData = MergeTWDataRecursive(
            nodeU->prev->prefixTwData, nodeX->postfixTwData);
        const double costSuppU
            = params.timeCost.get(nodeUPrevIndex, nodeXIndex)
              - params.timeCost.get(nodeUPrevIndex, nodeUIndex)
              - params.timeCost.get(nodeUIndex, nodeXIndex)
              + penaltyExcessLoad(routeU->load - loadU)
              + penaltyTimeWindows(routeUTwData) - routeU->penalty;

        for (Node *V = routeV->depot->next; !V->isDepot; V = V->next)
        {
            const TimeWindowData routeVTwData = MergeTWDataRecursive(
                V->prefixTwData, nodeU->twData, V->next->postfixTwData);
            double costSuppV = params.timeCost.get(V->cour, nodeUIndex)
                               + params.timeCost.get(nodeUIndex, V->next->cour)
                               - params.timeCost.get(V->cour, V->next->cour)
                               + penaltyExcessLoad(routeV->load + loadU)
                               + penaltyTimeWindows(routeVTwData)
                               - routeV->penalty;
            if (costSuppU + costSuppV < bestCost - MY_EPSILON)
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
    updateRouteData(routeU);
    updateRouteData(insertionPoint->route);

    return true;
}

int LocalSearch::getCheapestInsertSimultRemoval(Node *U,
                                                Node *V,
                                                Node *&bestPosition)
{
    ThreeBestInsert *myBestInsert = &bestInsertClient[V->route->cour][U->cour];
    bool found = false;

    // Find best insertion in the route such that V is not next or pred (can
    // only belong to the top three locations)

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
    int deltaCost = params.timeCost.get(V->prev->cour, U->cour)
                    + params.timeCost.get(U->cour, V->next->cour)
                    - params.timeCost.get(V->prev->cour, V->next->cour);
    if (!found || deltaCost < bestCost)
    {
        bestPosition = V->prev;
        bestCost = deltaCost;
    }

    return bestCost;
}

// TODO make this double as cost??
int LocalSearch::getCheapestInsertSimultRemovalWithTW(Node *U,
                                                      Node *V,
                                                      Node *&bestPosition)
{
    // TODO ThreeBestInsert must also use double as cost?
    ThreeBestInsert *myBestInsert
        = &bestInsertClientTW[V->route->cour][U->cour];
    bool found = false;

    // Find best insertion in the route such that V is not next or pred (can
    // only belong to the top three locations)
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
    TimeWindowData twData = MergeTWDataRecursive(
        V->prev->prefixTwData, U->twData, V->next->postfixTwData);
    int deltaCost = params.timeCost.get(V->prev->cour, U->cour)
                    + params.timeCost.get(U->cour, V->next->cour)
                    - params.timeCost.get(V->prev->cour, V->next->cour)
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
        U->deltaRemoval = params.timeCost.get(U->prev->cour, U->next->cour)
                          - params.timeCost.get(U->prev->cour, U->cour)
                          - params.timeCost.get(U->cour, U->next->cour);
        auto &currentOption = bestInsertClient[R2->cour][U->cour];
        if (R2->whenLastModified > currentOption.whenLastCalculated)
        {
            currentOption = ThreeBestInsert();
            currentOption.whenLastCalculated = nbMoves;
            currentOption.bestCost[0]
                = params.timeCost.get(0, U->cour)
                  + params.timeCost.get(U->cour, R2->depot->next->cour)
                  - params.timeCost.get(0, R2->depot->next->cour);
            currentOption.bestLocation[0] = R2->depot;
            for (Node *V = R2->depot->next; !V->isDepot; V = V->next)
            {
                int deltaCost = params.timeCost.get(V->cour, U->cour)
                                + params.timeCost.get(U->cour, V->next->cour)
                                - params.timeCost.get(V->cour, V->next->cour);
                currentOption.add(deltaCost, V);
            }
        }
    }
}

void LocalSearch::preprocessInsertionsWithTW(Route *R1, Route *R2)
{
    TimeWindowData twData;
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
            twData = MergeTWDataRecursive(U->prev->prefixTwData,
                                          U->next->postfixTwData);
            U->deltaRemovalTW
                = params.timeCost.get(U->prev->cour, U->next->cour)
                  - params.timeCost.get(U->prev->cour, U->cour)
                  - params.timeCost.get(U->cour, U->next->cour)
                  + penaltyTimeWindows(twData) - penaltyTimeWindows(R1->twData);
        }
        auto &currentOption = bestInsertClientTW[R2->cour][U->cour];
        if (R2->whenLastModified > currentOption.whenLastCalculated)
        {
            currentOption = ThreeBestInsert();
            currentOption.whenLastCalculated = nbMoves;

            // Compute additional timewarp we get when inserting U in R2, this
            // may be actually less if we remove U but we ignore this to have a
            // conservative estimate
            twData = MergeTWDataRecursive(R2->depot->prefixTwData,
                                          U->twData,
                                          R2->depot->next->postfixTwData);

            int cost = params.timeCost.get(0, U->cour)
                       + params.timeCost.get(U->cour, R2->depot->next->cour)
                       - params.timeCost.get(0, R2->depot->next->cour)
                       + penaltyTimeWindows(twData)
                       - penaltyTimeWindows(R2->twData);

            currentOption.add(cost, R2->depot);

            for (Node *V = R2->depot->next; !V->isDepot; V = V->next)
            {
                twData = MergeTWDataRecursive(
                    V->prefixTwData, U->twData, V->next->postfixTwData);

                int deltaCost = params.timeCost.get(V->cour, U->cour)
                                + params.timeCost.get(U->cour, V->next->cour)
                                - params.timeCost.get(V->cour, V->next->cour)
                                + penaltyTimeWindows(twData)
                                - penaltyTimeWindows(R2->twData);

                currentOption.add(deltaCost, V);
            }
        }
    }

    R1->isDeltaRemovalTWOutdated = false;
}

TimeWindowData LocalSearch::getEdgeTwData(Node *U, Node *V)
{
    // TODO this could be cached?
    return MergeTWDataRecursive(U->twData, V->twData);
}

TimeWindowData LocalSearch::getRouteSegmentTwData(Node *U, Node *V)
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
            twData = MergeTWDataRecursive(twData, mynode->toNextSeedTwD);
            mynode = mynode->nextSeed;
        }
        else
        {
            mynode = mynode->next;
            twData = MergeTWDataRecursive(twData, mynode->twData);
        }
    }
    return twData;
}

TimeWindowData LocalSearch::MergeTWDataRecursive(const TimeWindowData &twData1,
                                                 const TimeWindowData &twData2)
{
    TimeWindowData mergedTwData;
    // Note, assume time equals cost
    int deltaCost
        = params.timeCost.get(twData1.lastNodeIndex, twData2.firstNodeIndex);
    int deltaDuration = deltaCost;
    int delta = twData1.duration - twData1.timeWarp + deltaDuration;
    int deltaWaitTime = std::max(twData2.twEarly - delta - twData1.twLate, 0);
    int deltaTimeWarp = std::max(twData1.twEarly + delta - twData2.twLate, 0);
    mergedTwData.firstNodeIndex = twData1.firstNodeIndex;
    mergedTwData.lastNodeIndex = twData2.lastNodeIndex;
    mergedTwData.duration
        = twData1.duration + twData2.duration + deltaDuration + deltaWaitTime;
    mergedTwData.timeWarp = twData1.timeWarp + twData2.timeWarp + deltaTimeWarp;
    mergedTwData.twEarly
        = std::max(twData2.twEarly - delta, twData1.twEarly) - deltaWaitTime;
    mergedTwData.twLate
        = std::min(twData2.twLate - delta, twData1.twLate) + deltaTimeWarp;
    mergedTwData.latestReleaseTime
        = std::max(twData1.latestReleaseTime, twData2.latestReleaseTime);
    return mergedTwData;
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
        myload += params.cli[mynode->cour].demand;
        myReversalDistance
            += params.timeCost.get(mynode->cour, mynode->prev->cour)
               - params.timeCost.get(mynode->prev->cour, mynode->cour);
        mynode->cumulatedLoad = myload;
        mynode->cumulatedReversalDistance = myReversalDistance;
        mynode->prefixTwData
            = MergeTWDataRecursive(mynode->prev->prefixTwData, mynode->twData);
        mynode->isSeed = false;
        mynode->nextSeed = nullptr;
        if (!mynode->isDepot)
        {
            cumulatedX += params.cli[mynode->cour].x;
            cumulatedY += params.cli[mynode->cour].y;
            if (firstIt)
                myRoute->sector.initialize(params.cli[mynode->cour].angle);
            else
                myRoute->sector.extend(params.cli[mynode->cour].angle);

            if (myplace % 4 == 0)
            {
                if (seedNode != nullptr)
                {
                    seedNode->isSeed = true;
                    seedNode->toNextSeedTwD
                        = MergeTWDataRecursive(seedTwD, mynode->twData);
                    seedNode->nextSeed = mynode;
                }
                seedNode = mynode;
            }
            else if (myplace % 4 == 1)
                seedTwD = mynode->twData;
            else
                seedTwD = MergeTWDataRecursive(seedTwD, mynode->twData);
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
            = MergeTWDataRecursive(mynode->twData, mynode->next->postfixTwData);
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
                        - params.cli[0].y,
                    cumulatedX / static_cast<double>(myRoute->nbCustomers)
                        - params.cli[0].x);

        // Enforce minimum size of circle sector
        if (params.config.minCircleSectorSize > 0)
        {
            const int growSectorBy
                = (params.config.minCircleSectorSize
                   - myRoute->sector.positive_mod(myRoute->sector.end
                                                  - myRoute->sector.start)
                   + 1)
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

void LocalSearch::loadIndividual(Individual *indiv)
{
    emptyRoutes.clear();
    nbMoves = 0;
    TimeWindowData depotTwData;
    depotTwData.firstNodeIndex = 0;
    depotTwData.lastNodeIndex = 0;
    depotTwData.duration = 0;
    depotTwData.timeWarp = 0;
    depotTwData.twEarly = params.cli[0].twEarly;
    depotTwData.twLate = params.cli[0].twLate;
    depotTwData.latestReleaseTime = params.cli[0].releaseTime;

    // Initializing time window data (before loop since it is needed in update
    // route)
    for (int i = 1; i <= params.nbClients; i++)
    {
        TimeWindowData *myTwData = &clients[i].twData;
        myTwData->firstNodeIndex = i;
        myTwData->lastNodeIndex = i;
        myTwData->duration = params.cli[i].servDur;
        myTwData->twEarly = params.cli[i].twEarly;
        myTwData->twLate = params.cli[i].twLate;
        myTwData->latestReleaseTime = params.cli[i].releaseTime;
        // myTwData->load = params.cli[i].demand;
    }

    auto const &routesIndiv = indiv->getRoutes();

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

    // empty routes have a polar angle of 1.e30, and therefore will always
    // appear at the end
    std::sort(routePolarAngles.begin(), routePolarAngles.end());

    std::vector<int> tour(params.nbClients);
    std::vector<std::vector<int>> routes(params.nbVehicles);

    int pos = 0;
    for (int r = 0; r < params.nbVehicles; r++)
    {
        Node *node = depots[routePolarAngles[r].second].next;

        while (!node->isDepot)
        {
            tour[pos] = node->cour;
            routes[r].push_back(node->cour);
            node = node->next;
            pos++;
        }
    }

    return {&params, tour, routes};
}

LocalSearch::LocalSearch(Params &params, XorShift128 &rng)
    : params(params), rng(rng)
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
    for (int i = 1; i <= params.nbClients; i++)
        orderNodes.push_back(i);
    for (int r = 0; r < params.nbVehicles; r++)
        orderRoutes.push_back(r);
}
