#include "operators.h"

#include <cmath>

void operators::insertNode(LocalSearch::Node *U, LocalSearch::Node *V)
{
    U->prev->next = U->next;
    U->next->prev = U->prev;
    V->next->prev = U;
    U->prev = V;
    U->next = V->next;
    V->next = U;
    U->route = V->route;
}

void operators::swapNode(LocalSearch::Node *U, LocalSearch::Node *V)
{
    auto *myVPred = V->prev;
    auto *myVSuiv = V->next;
    auto *myUPred = U->prev;
    auto *myUSuiv = U->next;
    auto *myRouteU = U->route;
    auto *myRouteV = V->route;

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

void operators::updateRouteData(LocalSearch::Route *myRoute,
                                int nbMoves,
                                LocalSearch::Penalties const &penalties,
                                Params const &params)
{
    int myplace = 0;
    int myload = 0;
    int myReversalDistance = 0;
    int cumulatedX = 0;
    int cumulatedY = 0;

    auto *mynode = myRoute->depot;
    mynode->position = 0;
    mynode->cumulatedLoad = 0;
    mynode->cumulatedReversalDistance = 0;

    bool firstIt = true;
    auto seedTwD = mynode->twData;
    LocalSearch::Node *seedNode = nullptr;

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
        mynode->prefixTwData = mynode->prev->prefixTwData.merge(mynode->twData);
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
                    seedNode->toNextSeedTwD = seedTwD.merge(mynode->twData);
                    seedNode->nextSeed = mynode;
                }
                seedNode = mynode;
            }
            else if (myplace % 4 == 1)
                seedTwD = mynode->twData;
            else
                seedTwD = seedTwD.merge(mynode->twData);
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
        mynode->postfixTwData = LocalSearch::TimeWindowData::merge(
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
