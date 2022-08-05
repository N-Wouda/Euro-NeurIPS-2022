#include "LocalSearch.h"

#include "CircleSector.h"
#include "Individual.h"
#include "Params.h"
#include "TimeWindowSegment.h"
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
            for (auto const &v : params.getNeighboursOf(nodeU->client))
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
                    if (moveSingleClient(
                            nbMoves, searchCompleted, nodeU, nodeV, penalties))
                        continue;  // RELOCATE
                    if (moveTwoClients(
                            nbMoves, searchCompleted, nodeU, nodeV, penalties))
                        continue;  // RELOCATE
                    if (moveTwoClientsReversed(
                            nbMoves, searchCompleted, nodeU, nodeV, penalties))
                        continue;  // RELOCATE
                    if (swapTwoSingleClients(
                            nbMoves, searchCompleted, nodeU, nodeV, penalties))
                        continue;  // SWAP
                    if (swapTwoClientsForOne(
                            nbMoves, searchCompleted, nodeU, nodeV, penalties))
                        continue;  // SWAP
                    if (swapTwoClientPairs(
                            nbMoves, searchCompleted, nodeU, nodeV, penalties))
                        continue;  // SWAP
                    if (twoOptBetweenTrips(
                            nbMoves, searchCompleted, nodeU, nodeV, penalties))
                        continue;  // 2-OPT*
                    if (twoOptWithinTrip(
                            nbMoves, searchCompleted, nodeU, nodeV, penalties))
                        continue;  // 2-OPT

                    // Trying moves that insert nodeU directly after the depot
                    if (nodeV->prev->isDepot)
                    {
                        nodeV = nodeV->prev;

                        if (moveSingleClient(nbMoves,
                                             searchCompleted,
                                             nodeU,
                                             nodeV,
                                             penalties))
                            continue;  // RELOCATE
                        if (moveTwoClients(nbMoves,
                                           searchCompleted,
                                           nodeU,
                                           nodeV,
                                           penalties))
                            continue;  // RELOCATE
                        if (moveTwoClientsReversed(nbMoves,
                                                   searchCompleted,
                                                   nodeU,
                                                   nodeV,
                                                   penalties))
                            continue;  // RELOCATE
                        if (twoOptBetweenTrips(nbMoves,
                                               searchCompleted,
                                               nodeU,
                                               nodeV,
                                               penalties))
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

                if (moveSingleClient(
                        nbMoves, searchCompleted, nodeU, nodeV, penalties))
                    continue;  // RELOCATE
                if (moveTwoClients(
                        nbMoves, searchCompleted, nodeU, nodeV, penalties))
                    continue;  // RELOCATE
                if (moveTwoClientsReversed(
                        nbMoves, searchCompleted, nodeU, nodeV, penalties))
                    continue;  // RELOCATE
                if (twoOptBetweenTrips(
                        nbMoves, searchCompleted, nodeU, nodeV, penalties))
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

                    if (routeV->nbCustomers == 0 || routeU->idx >= routeV->idx)
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

                    if (relocateStar(nbMoves,
                                     searchCompleted,
                                     routeU,
                                     routeV,
                                     penalties))
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

bool LocalSearch::swapStar(bool const withTW,
                           int &nbMoves,
                           bool &searchCompleted,
                           Route *routeU,
                           Route *routeV)
{
    SwapStarElement myBestSwapStar;

    if (!bestInsertInitializedForRoute[routeU->idx])
    {
        bestInsertInitializedForRoute[routeU->idx] = true;
        for (int i = 1; i <= params.nbClients; i++)
        {
            bestInsertClient[routeU->idx][i].whenLastCalculated = -1;
            bestInsertClientTW[routeU->idx][i].whenLastCalculated = -1;
        }
    }
    if (!bestInsertInitializedForRoute[routeV->idx])
    {
        bestInsertInitializedForRoute[routeV->idx] = true;
        for (int i = 1; i <= params.nbClients; i++)
        {
            bestInsertClient[routeV->idx][i].whenLastCalculated = -1;
            bestInsertClientTW[routeV->idx][i].whenLastCalculated = -1;
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
                routeU->load + params.clients[second->client].demand
                - params.clients[first->client].demand);
            int const loadPenV = penalties.load(
                routeV->load + params.clients[first->client].demand
                - params.clients[second->client].demand);
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
    int costSuppU = params.dist(myBestSwapStar.bestPositionV->client,
                                myBestSwapStar.V->client)
                    - params.dist(myBestSwapStar.U->prev->client,
                                  myBestSwapStar.U->client)
                    - params.dist(myBestSwapStar.U->client,
                                  myBestSwapStar.U->next->client);
    int costSuppV = params.dist(myBestSwapStar.bestPositionU->client,
                                myBestSwapStar.U->client)
                    - params.dist(myBestSwapStar.V->prev->client,
                                  myBestSwapStar.V->client)
                    - params.dist(myBestSwapStar.V->client,
                                  myBestSwapStar.V->next->client);

    if (myBestSwapStar.bestPositionV == myBestSwapStar.U->prev)
    {
        // Insert in place of U
        costSuppU += params.dist(myBestSwapStar.V->client,
                                 myBestSwapStar.U->next->client);
    }
    else
    {
        costSuppU += params.dist(myBestSwapStar.V->client,
                                 myBestSwapStar.bestPositionV->next->client)
                     + params.dist(myBestSwapStar.U->prev->client,
                                   myBestSwapStar.U->next->client)
                     - params.dist(myBestSwapStar.bestPositionV->client,
                                   myBestSwapStar.bestPositionV->next->client);
    }

    if (myBestSwapStar.bestPositionU == myBestSwapStar.V->prev)
    {
        // Insert in place of V
        costSuppV += params.dist(myBestSwapStar.U->client,
                                 myBestSwapStar.V->next->client);
    }
    else
    {
        costSuppV += params.dist(myBestSwapStar.U->client,
                                 myBestSwapStar.bestPositionU->next->client)
                     + params.dist(myBestSwapStar.V->prev->client,
                                   myBestSwapStar.V->next->client)
                     - params.dist(myBestSwapStar.bestPositionU->client,
                                   myBestSwapStar.bestPositionU->next->client);
    }

    // It is not possible to have bestPositionU == V or bestPositionV == U, so
    // the positions are always strictly different
    if (myBestSwapStar.bestPositionV->position
        == myBestSwapStar.U->position - 1)
    {
        // Special case
        auto const routeUTwData
            = TimeWindowSegment::merge(myBestSwapStar.bestPositionV->twBefore,
                                       myBestSwapStar.V->tw,
                                       myBestSwapStar.U->next->twAfter);

        costSuppU += penalties.timeWarp(routeUTwData);
    }
    else if (myBestSwapStar.bestPositionV->position
             < myBestSwapStar.U->position)
    {
        auto const routeUTwData = TimeWindowSegment::merge(
            myBestSwapStar.bestPositionV->twBefore,
            myBestSwapStar.V->tw,
            myBestSwapStar.bestPositionV->next->mergeSegmentTwData(
                myBestSwapStar.U->prev),
            myBestSwapStar.U->next->twAfter);

        costSuppU += penalties.timeWarp(routeUTwData);
    }
    else
    {
        auto const routeUTwData = TimeWindowSegment::merge(
            myBestSwapStar.U->prev->twBefore,
            myBestSwapStar.U->next->mergeSegmentTwData(
                myBestSwapStar.bestPositionV),
            myBestSwapStar.V->tw,
            myBestSwapStar.bestPositionV->next->twAfter);

        costSuppU += penalties.timeWarp(routeUTwData);
    }

    if (myBestSwapStar.bestPositionU->position
        == myBestSwapStar.V->position - 1)
    {
        // Special case
        auto const routeVTwData
            = TimeWindowSegment::merge(myBestSwapStar.bestPositionU->twBefore,
                                       myBestSwapStar.U->tw,
                                       myBestSwapStar.V->next->twAfter);

        costSuppV += penalties.timeWarp(routeVTwData);
    }
    else if (myBestSwapStar.bestPositionU->position
             < myBestSwapStar.V->position)
    {
        auto const routeVTwData = TimeWindowSegment::merge(
            myBestSwapStar.bestPositionU->twBefore,
            myBestSwapStar.U->tw,
            myBestSwapStar.bestPositionU->next->mergeSegmentTwData(
                myBestSwapStar.V->prev),
            myBestSwapStar.V->next->twAfter);

        costSuppV += penalties.timeWarp(routeVTwData);
    }
    else
    {
        auto const routeVTwData = TimeWindowSegment::merge(
            myBestSwapStar.V->prev->twBefore,
            myBestSwapStar.V->next->mergeSegmentTwData(
                myBestSwapStar.bestPositionU),
            myBestSwapStar.U->tw,
            myBestSwapStar.bestPositionU->next->twAfter);

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

int LocalSearch::getCheapestInsertSimultRemoval(Node *U,
                                                Node *V,
                                                Node *&bestPosition)
{
    ThreeBestInsert *myBestInsert = &bestInsertClient[V->route->idx][U->client];
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
    int deltaCost = params.dist(V->prev->client, U->client)
                    + params.dist(U->client, V->next->client)
                    - params.dist(V->prev->client, V->next->client);
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
        = &bestInsertClientTW[V->route->idx][U->client];
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
    TimeWindowSegment twData
        = TimeWindowSegment::merge(V->prev->twBefore, U->tw, V->next->twAfter);
    int deltaCost = params.dist(V->prev->client, U->client)
                    + params.dist(U->client, V->next->client)
                    - params.dist(V->prev->client, V->next->client)
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
        U->deltaRemoval = params.dist(U->prev->client, U->next->client)
                          - params.dist(U->prev->client, U->client)
                          - params.dist(U->client, U->next->client);
        auto &currentOption = bestInsertClient[R2->idx][U->client];
        if (R2->whenLastModified > currentOption.whenLastCalculated)
        {
            currentOption = ThreeBestInsert();
            currentOption.whenLastCalculated = nbMoves;
            currentOption.bestCost[0]
                = params.dist(0, U->client)
                  + params.dist(U->client, R2->depot->next->client)
                  - params.dist(0, R2->depot->next->client);
            currentOption.bestLocation[0] = R2->depot;
            for (Node *V = R2->depot->next; !V->isDepot; V = V->next)
            {
                int deltaCost = params.dist(V->client, U->client)
                                + params.dist(U->client, V->next->client)
                                - params.dist(V->client, V->next->client);
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
            auto const twData
                = TimeWindowSegment::merge(U->prev->twBefore, U->next->twAfter);
            U->deltaRemovalTW = params.dist(U->prev->client, U->next->client)
                                - params.dist(U->prev->client, U->client)
                                - params.dist(U->client, U->next->client)
                                + penalties.timeWarp(twData)
                                - penalties.timeWarp(R1->twData);
        }
        auto &currentOption = bestInsertClientTW[R2->idx][U->client];
        if (R2->whenLastModified > currentOption.whenLastCalculated)
        {
            currentOption = ThreeBestInsert();
            currentOption.whenLastCalculated = nbMoves;

            // Compute additional timewarp we get when inserting U in R2, this
            // may be actually less if we remove U but we ignore this to have a
            // conservative estimate
            auto twData = TimeWindowSegment::merge(
                R2->depot->twBefore, U->tw, R2->depot->next->twAfter);

            int cost = params.dist(0, U->client)
                       + params.dist(U->client, R2->depot->next->client)
                       - params.dist(0, R2->depot->next->client)
                       + penalties.timeWarp(twData)
                       - penalties.timeWarp(R2->twData);

            currentOption.add(cost, R2->depot);

            for (Node *V = R2->depot->next; !V->isDepot; V = V->next)
            {
                twData = TimeWindowSegment::merge(
                    V->twBefore, U->tw, V->next->twAfter);

                int deltaCost = params.dist(V->client, U->client)
                                + params.dist(U->client, V->next->client)
                                - params.dist(V->client, V->next->client)
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
    TimeWindowSegment seedTwD = mynode->tw;
    Node *seedNode = nullptr;
    while (!mynode->isDepot || firstIt)
    {
        mynode = mynode->next;
        myplace++;
        mynode->position = myplace;
        myload += params.clients[mynode->client].demand;
        myReversalDistance
            += params.dist(mynode->client, mynode->prev->client)
               - params.dist(mynode->prev->client, mynode->client);
        mynode->cumulatedLoad = myload;
        mynode->cumulatedReversalDistance = myReversalDistance;
        mynode->twBefore
            = TimeWindowSegment::merge(mynode->prev->twBefore, mynode->tw);
        mynode->isSeed = false;
        mynode->nextSeed = nullptr;
        if (!mynode->isDepot)
        {
            cumulatedX += params.clients[mynode->client].x;
            cumulatedY += params.clients[mynode->client].y;
            if (firstIt)
                myRoute->sector.initialize(
                    params.clients[mynode->client].angle);
            else
                myRoute->sector.extend(params.clients[mynode->client].angle);

            if (myplace % 4 == 0)
            {
                if (seedNode != nullptr)
                {
                    seedNode->isSeed = true;
                    seedNode->toNextSeedTwD
                        = TimeWindowSegment::merge(seedTwD, mynode->tw);
                    seedNode->nextSeed = mynode;
                }
                seedNode = mynode;
            }
            else if (myplace % 4 == 1)
                seedTwD = mynode->tw;
            else
                seedTwD = TimeWindowSegment::merge(seedTwD, mynode->tw);
        }
        firstIt = false;
    }

    myRoute->load = myload;
    myRoute->twData = mynode->twBefore;
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
        mynode->twAfter
            = TimeWindowSegment::merge(mynode->tw, mynode->next->twAfter);
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
    TimeWindowSegment depotTwData = {&params,
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
        clients[i].tw = {&params,
                         i,
                         i,
                         params.clients[i].servDur,
                         0,
                         params.clients[i].twEarly,
                         params.clients[i].twLate,
                         params.clients[i].releaseTime};
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

        myDepot->tw = depotTwData;
        myDepot->twBefore = depotTwData;
        myDepot->twAfter = depotTwData;
        myDepot->isSeed = false;

        myDepotFin->tw = depotTwData;
        myDepotFin->twBefore = depotTwData;
        myDepotFin->twAfter = depotTwData;
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
            indivRoutes[r].push_back(node->client);
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
        clients[i].params = &params;
        clients[i].client = i;
        clients[i].isDepot = false;
    }

    for (int i = 0; i < params.nbVehicles; i++)
    {
        routes[i].params = &params;
        routes[i].idx = i;
        routes[i].depot = &depots[i];
        depots[i].client = 0;
        depots[i].isDepot = true;
        depots[i].route = &routes[i];
        depotsEnd[i].client = 0;
        depotsEnd[i].isDepot = true;
        depotsEnd[i].route = &routes[i];
    }

    std::iota(orderNodes.begin(), orderNodes.end(), 1);
    std::iota(orderRoutes.begin(), orderRoutes.end(), 0);
}
