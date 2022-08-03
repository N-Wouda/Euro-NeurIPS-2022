#include "LocalSearch.h"

#include "CircleSector.h"
#include "Individual.h"
#include "Params.h"
#include "operators.h"

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

    for (int step = 0; !searchCompleted; ++step)
    {
        if (step > 1)                // At least two loops as some moves with
            searchCompleted = true;  // empty routes are not done in the first

        /* ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY RESTRICTION */
        for (int posU = 0; posU < params.nbClients; posU++)
        {
            Node *nodeU = &clients[orderNodes[posU]];
            int lastTestRINodeU = nodeU->whenLastTestedRI;
            nodeU->whenLastTestedRI = nbMoves;

            // Randomizing the order of the neighborhoods within this loop does
            // not matter much as we are already randomizing the order of the
            // node pairs (and it's not very common to find improving moves of
            // different types for the same node pair)
            for (auto const &v : params.getNeighboursOf(nodeU->cour))
            {
                Node *nodeV = &clients[v];

                if (step == 0
                    || std::max(nodeU->route->whenLastModified,
                                nodeV->route->whenLastModified)
                           > lastTestRINodeU)  // only evaluate moves involving
                                               // routes that have been
                                               // modified since last move
                                               // evaluations for nodeU
                {
                    if (MoveSingleClient(
                            nbMoves, searchCompleted, nodeU, nodeV))
                        continue;  // RELOCATE
                    if (MoveTwoClients(nbMoves, searchCompleted, nodeU, nodeV))
                        continue;  // RELOCATE
                    if (MoveTwoClientsReversed(
                            nbMoves, searchCompleted, nodeU, nodeV))
                        continue;  // RELOCATE
                    if (SwapTwoSingleClients(
                            nbMoves, searchCompleted, nodeU, nodeV))
                        continue;  // SWAP
                    if (SwapTwoClientsForOne(
                            nbMoves, searchCompleted, nodeU, nodeV))
                        continue;  // SWAP
                    if (SwapTwoClientPairs(
                            nbMoves, searchCompleted, nodeU, nodeV))
                        continue;  // SWAP
                    if (TwoOptBetweenTrips(
                            nbMoves, searchCompleted, nodeU, nodeV))
                        continue;  // 2-OPT*
                    if (TwoOptWithinTrip(
                            nbMoves, searchCompleted, nodeU, nodeV))
                        continue;  // 2-OPT

                    // Trying moves that insert nodeU directly after the depot
                    if (nodeV->prev->isDepot)
                    {
                        nodeV = nodeV->prev;

                        if (MoveSingleClient(
                                nbMoves, searchCompleted, nodeU, nodeV))
                            continue;  // RELOCATE
                        if (MoveTwoClients(
                                nbMoves, searchCompleted, nodeU, nodeV))
                            continue;  // RELOCATE
                        if (MoveTwoClientsReversed(
                                nbMoves, searchCompleted, nodeU, nodeV))
                            continue;  // RELOCATE
                        if (TwoOptBetweenTrips(
                                nbMoves, searchCompleted, nodeU, nodeV))
                            continue;  // 2-OPT*
                    }
                }
            }

            /* MOVES INVOLVING AN EMPTY ROUTE -- NOT TESTED IN THE FIRST LOOP TO
             * AVOID INCREASING TOO MUCH THE FLEET SIZE */
            if (step > 0)
            {
                auto pred = [](auto &route) { return route.nbCustomers == 0; };
                auto empty = std::find_if(routes.begin(), routes.end(), pred);

                if (empty == routes.end())
                    continue;

                Node *nodeV = empty->depot;

                if (MoveSingleClient(nbMoves, searchCompleted, nodeU, nodeV))
                    continue;  // RELOCATE
                if (MoveTwoClients(nbMoves, searchCompleted, nodeU, nodeV))
                    continue;  // RELOCATE
                if (MoveTwoClientsReversed(
                        nbMoves, searchCompleted, nodeU, nodeV))
                    continue;  // RELOCATE
                if (TwoOptBetweenTrips(nbMoves, searchCompleted, nodeU, nodeV))
                    continue;  // 2-OPT*
            }
        }

        /* (SWAP*) MOVES LIMITED TO ROUTE PAIRS WHOSE CIRCLE SECTORS OVERLAP */
        // TODO use node pairs instead?
        if (searchCompleted && shouldIntensify)
        {
            for (int rU = 0; rU < params.nbVehicles; rU++)
            {
                Route *routeU = &routes[orderRoutes[rU]];
                if (routeU->nbCustomers == 0)
                    continue;

                int lastTestLargeNbRouteU = routeU->whenLastTestedLargeNb;
                routeU->whenLastTestedLargeNb = nbMoves;

                for (int rV = 0; rV < params.nbVehicles; rV++)
                {
                    Route *routeV = &routes[orderRoutes[rV]];

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

                    if (RelocateStar(nbMoves, searchCompleted, routeU, routeV))
                        continue;
                    if (swapStar(
                            false, nbMoves, searchCompleted, routeU, routeV))
                        continue;
                    if (swapStar(
                            true, nbMoves, searchCompleted, routeU, routeV))
                        continue;
                }
            }
        }
    }
}

bool LocalSearch::MoveSingleClient(int &nbMoves,
                                   bool &searchCompleted,
                                   Node *nodeU,
                                   Node *nodeV)
{
    return moveSingleClient(
        nbMoves, searchCompleted, nodeU, nodeV, penalties, params);
}

bool LocalSearch::MoveTwoClients(int &nbMoves,
                                 bool &searchCompleted,
                                 Node *nodeU,
                                 Node *nodeV)
{
    return moveTwoClients(
        nbMoves, searchCompleted, nodeU, nodeV, penalties, params);
}

bool LocalSearch::MoveTwoClientsReversed(int &nbMoves,
                                         bool &searchCompleted,
                                         Node *nodeU,
                                         Node *nodeV)
{
    return moveTwoClientsReversed(
        nbMoves, searchCompleted, nodeU, nodeV, penalties, params);
}

bool LocalSearch::SwapTwoSingleClients(int &nbMoves,
                                       bool &searchCompleted,
                                       Node *nodeU,
                                       Node *nodeV)
{
    return swapTwoSingleClients(
        nbMoves, searchCompleted, nodeU, nodeV, penalties, params);
}

bool LocalSearch::SwapTwoClientsForOne(int &nbMoves,
                                       bool &searchCompleted,
                                       Node *nodeU,
                                       Node *nodeV)
{
    return swapTwoClientsForOne(
        nbMoves, searchCompleted, nodeU, nodeV, penalties, params);
}

bool LocalSearch::SwapTwoClientPairs(int &nbMoves,
                                     bool &searchCompleted,
                                     Node *nodeU,
                                     Node *nodeV)
{
    return swapTwoClientPairs(
        nbMoves, searchCompleted, nodeU, nodeV, penalties, params);
}

bool LocalSearch::TwoOptWithinTrip(int &nbMoves,
                                   bool &searchCompleted,
                                   Node *nodeU,
                                   Node *nodeV)
{
    if (nodeU->route != nodeV->route)
        return false;

    if (nodeU->position >= nodeV->position - 1)
        return false;

    int cost = params.dist(nodeU->cour, nodeV->cour)
               + params.dist(nodeU->next->cour, nodeV->next->cour)
               - params.dist(nodeU->cour, nodeU->next->cour)
               - params.dist(nodeV->cour, nodeV->next->cour)
               + nodeV->cumulatedReversalDistance
               - nodeU->next->cumulatedReversalDistance;

    if (nodeU->route->twData.timeWarp == 0 && cost >= 0)
    {
        return false;
    }

    TimeWindowData routeTwData = nodeU->prefixTwData;
    Node *itRoute = nodeV;
    while (itRoute != nodeU)
    {
        routeTwData = TimeWindowData::merge(routeTwData, itRoute->twData);
        itRoute = itRoute->prev;
    }
    routeTwData
        = TimeWindowData::merge(routeTwData, nodeV->next->postfixTwData);

    // Compute new total penalty
    cost += penalties.load(nodeU->route->load) + penalties.timeWarp(routeTwData)
            - nodeU->route->penalty;

    if (cost >= 0)
    {
        return false;
    }

    itRoute = nodeV;
    auto *insertionPoint = nodeU;
    auto *currNext = nodeU->next;

    while (itRoute != currNext)  // No need to move x, we pivot around it
    {
        Node *current = itRoute;
        itRoute = itRoute->prev;
        insertNode(current, insertionPoint);
        insertionPoint = current;
    }

    nbMoves++;
    searchCompleted = false;
    updateRouteData(nodeU->route, nbMoves);

    return true;
}

bool LocalSearch::TwoOptBetweenTrips(int &nbMoves,
                                     bool &searchCompleted,
                                     Node *nodeU,
                                     Node *nodeV)
{
    if (nodeU->route->cour >= nodeV->route->cour)
        return false;

    int costSuppU = params.dist(nodeU->cour, nodeV->next->cour)
                    - params.dist(nodeU->cour, nodeU->next->cour);
    int costSuppV = params.dist(nodeV->cour, nodeU->next->cour)
                    - params.dist(nodeV->cour, nodeV->next->cour);

    if (nodeU->route->load <= params.vehicleCapacity
        && nodeU->route->twData.timeWarp == 0
        && nodeV->route->load <= params.vehicleCapacity
        && nodeV->route->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
    {
        return false;
    }

    auto routeUTwData = TimeWindowData::merge(nodeU->prefixTwData,
                                              nodeV->next->postfixTwData);
    auto routeVTwData = TimeWindowData::merge(nodeV->prefixTwData,
                                              nodeU->next->postfixTwData);

    costSuppU += penalties.load(nodeU->cumulatedLoad + nodeV->route->load
                                - nodeV->cumulatedLoad)
                 + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

    costSuppV += penalties.load(nodeV->cumulatedLoad + nodeU->route->load
                                - nodeU->cumulatedLoad)
                 + penalties.timeWarp(routeVTwData) - nodeV->route->penalty;

    if (costSuppU + costSuppV >= 0)
        return false;

    auto *routeU = nodeU->route;  // these values change in the following, so
    auto *routeV = nodeV->route;  // must be set here already.
    auto *itRouteU = nodeU->next;
    auto *itRouteV = nodeV->next;

    Node *insertLocation = nodeU;
    while (!itRouteV->isDepot)
    {
        Node *current = itRouteV;
        itRouteV = itRouteV->next;
        insertNode(current, insertLocation);
        insertLocation = current;
    }

    insertLocation = nodeV;
    while (!itRouteU->isDepot)
    {
        Node *current = itRouteU;
        itRouteU = itRouteU->next;
        insertNode(current, insertLocation);
        insertLocation = current;
    }

    nbMoves++;
    searchCompleted = false;
    updateRouteData(routeU, nbMoves);
    updateRouteData(routeV, nbMoves);

    return true;
}

bool LocalSearch::swapStar(bool const withTW,
                           int &nbMoves,
                           bool &searchCompleted,
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
    for (Node *first = routeU->depot->next; !first->isDepot;
         first = first->next)
    {
        for (Node *second = routeV->depot->next; !second->isDepot;
             second = second->next)
        {
            // We cannot determine impact on timewarp without adding too much
            // complexity (O(n^3) instead of O(n^2))
            int const loadPenU = penalties.load(
                routeU->load + params.clients[second->cour].demand
                - params.clients[first->cour].demand);
            int const loadPenV = penalties.load(
                routeV->load + params.clients[first->cour].demand
                - params.clients[second->cour].demand);
            int const deltaLoadPen = loadPenU + loadPenV
                                     - penalties.load(routeU->load)
                                     - penalties.load(routeV->load);
            const int deltaRemoval
                = withTW ? first->deltaRemovalTW + second->deltaRemovalTW
                         : first->deltaRemoval + second->deltaRemoval;

            // Quick filter: possibly early elimination of many SWAP* due to the
            // capacity constraints/penalties and bounds on insertion costs
            if (deltaLoadPen + deltaRemoval <= 0)
            {
                SwapStarElement mySwapStar;
                mySwapStar.U = first;
                mySwapStar.V = second;

                int extraV, extraU;
                if (withTW)
                {
                    // Evaluate best reinsertion cost of U in the route of V
                    // where V has been removed
                    extraV = getCheapestInsertSimultRemovalWithTW(
                        first, second, mySwapStar.bestPositionU);

                    // Evaluate best reinsertion cost of V in the route of U
                    // where U has been removed
                    extraU = getCheapestInsertSimultRemovalWithTW(
                        second, first, mySwapStar.bestPositionV);
                }
                else
                {
                    // Evaluate best reinsertion cost of U in the route of V
                    // where V has been removed
                    extraV = getCheapestInsertSimultRemoval(
                        first, second, mySwapStar.bestPositionU);

                    // Evaluate best reinsertion cost of V in the route of U
                    // where U has been removed
                    extraU = getCheapestInsertSimultRemoval(
                        second, first, mySwapStar.bestPositionV);
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
            = TimeWindowData::merge(myBestSwapStar.bestPositionV->prefixTwData,
                                    myBestSwapStar.V->twData,
                                    myBestSwapStar.U->next->postfixTwData);

        costSuppU += penalties.timeWarp(routeUTwData);
    }
    else if (myBestSwapStar.bestPositionV->position
             < myBestSwapStar.U->position)
    {
        auto const routeUTwData = TimeWindowData::merge(
            myBestSwapStar.bestPositionV->prefixTwData,
            myBestSwapStar.V->twData,
            myBestSwapStar.bestPositionV->next->mergeSegmentTwData(
                myBestSwapStar.U->prev),
            myBestSwapStar.U->next->postfixTwData);

        costSuppU += penalties.timeWarp(routeUTwData);
    }
    else
    {
        auto const routeUTwData = TimeWindowData::merge(
            myBestSwapStar.U->prev->prefixTwData,
            myBestSwapStar.U->next->mergeSegmentTwData(
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
            = TimeWindowData::merge(myBestSwapStar.bestPositionU->prefixTwData,
                                    myBestSwapStar.U->twData,
                                    myBestSwapStar.V->next->postfixTwData);

        costSuppV += penalties.timeWarp(routeVTwData);
    }
    else if (myBestSwapStar.bestPositionU->position
             < myBestSwapStar.V->position)
    {
        auto const routeVTwData = TimeWindowData::merge(
            myBestSwapStar.bestPositionU->prefixTwData,
            myBestSwapStar.U->twData,
            myBestSwapStar.bestPositionU->next->mergeSegmentTwData(
                myBestSwapStar.V->prev),
            myBestSwapStar.V->next->postfixTwData);

        costSuppV += penalties.timeWarp(routeVTwData);
    }
    else
    {
        auto const routeVTwData = TimeWindowData::merge(
            myBestSwapStar.V->prev->prefixTwData,
            myBestSwapStar.V->next->mergeSegmentTwData(
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
                               Route *routeU,
                               Route *routeV)
{
    int bestCost = 0;
    Node *insertionPoint = nullptr;
    Node *nodeToInsert = nullptr;
    for (Node *nodeU = routeU->depot->next; !nodeU->isDepot;
         nodeU = nodeU->next)
    {
        routeU = nodeU->route;
        Node *nodeX = nodeU->next;

        TimeWindowData const routeUTwData = TimeWindowData::merge(
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
            TimeWindowData const routeVTwData = TimeWindowData::merge(
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
    TimeWindowData twData = TimeWindowData::merge(
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
            auto const twData = TimeWindowData::merge(U->prev->prefixTwData,
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
            auto twData = TimeWindowData::merge(R2->depot->prefixTwData,
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
                twData = TimeWindowData::merge(
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
            = TimeWindowData::merge(mynode->prev->prefixTwData, mynode->twData);
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
                        = TimeWindowData::merge(seedTwD, mynode->twData);
                    seedNode->nextSeed = mynode;
                }
                seedNode = mynode;
            }
            else if (myplace % 4 == 1)
                seedTwD = mynode->twData;
            else
                seedTwD = TimeWindowData::merge(seedTwD, mynode->twData);
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
        mynode->postfixTwData = TimeWindowData::merge(
            mynode->twData, mynode->next->postfixTwData);
    } while (!mynode->isDepot);

    if (myRoute->nbCustomers == 0)
    {
        myRoute->polarAngleBarycenter = 1.e30;
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
    }
}

void LocalSearch::loadIndividual(Individual const &indiv)
{
    TimeWindowData depotTwData = {&params,
                                  0,
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
        myTwData->params = &params;
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
