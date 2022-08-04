#include "operators.h"

#include "TimeWindowSegment.h"

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

void operators::updateRouteData(LocalSearch::Route *route,
                                int nbMoves,
                                LocalSearch::Penalties const &penalties,
                                Params const &params)
{
    int place = 0;
    int load = 0;
    int reverseDistance = 0;
    int cumulatedX = 0;
    int cumulatedY = 0;

    auto *node = route->depot;
    node->position = 0;
    node->cumulatedLoad = 0;
    node->cumulatedReversalDistance = 0;

    bool firstIt = true;
    auto seedTwD = node->tw;
    LocalSearch::Node *seedNode = nullptr;

    while (!node->isDepot || firstIt)
    {
        node = node->next;
        place++;
        node->position = place;
        load += params.clients[node->client].demand;
        reverseDistance += params.dist(node->client, node->prev->client)
                           - params.dist(node->prev->client, node->client);
        node->cumulatedLoad = load;
        node->cumulatedReversalDistance = reverseDistance;
        node->twBefore
            = TimeWindowSegment::merge(node->prev->twBefore, node->tw);
        node->isSeed = false;
        node->nextSeed = nullptr;
        if (!node->isDepot)
        {
            cumulatedX += params.clients[node->client].x;
            cumulatedY += params.clients[node->client].y;
            if (firstIt)
                route->sector.initialize(params.clients[node->client].angle);
            else
                route->sector.extend(params.clients[node->client].angle);

            if (place % 4 == 0)
            {
                if (seedNode != nullptr)
                {
                    seedNode->isSeed = true;
                    seedNode->toNextSeedTwD
                        = TimeWindowSegment::merge(seedTwD, node->tw);
                    seedNode->nextSeed = node;
                }
                seedNode = node;
            }
            else if (place % 4 == 1)
                seedTwD = node->tw;
            else
                seedTwD = TimeWindowSegment::merge(seedTwD, node->tw);
        }
        firstIt = false;
    }

    route->load = load;
    route->twData = node->twBefore;
    route->penalty = penalties.load(load) + penalties.timeWarp(route->twData);
    route->nbCustomers = place - 1;
    // Remember "when" this route has been last modified (will be used to filter
    // unnecessary move evaluations)
    route->whenLastModified = nbMoves;
    route->isDeltaRemovalTWOutdated = true;

    // Time window data in reverse direction, node should be end depot now
    do
    {
        node = node->prev;
        node->twAfter = TimeWindowSegment::merge(node->tw, node->next->twAfter);
    } while (!node->isDepot);

    if (route->nbCustomers == 0)
    {
        route->polarAngleBarycenter = 1.e30;
    }
    else
    {
        route->polarAngleBarycenter
            = atan2(cumulatedY / static_cast<double>(route->nbCustomers)
                        - params.clients[0].y,
                    cumulatedX / static_cast<double>(route->nbCustomers)
                        - params.clients[0].x);

        // Enforce minimum size of circle sector
        if (params.config.minCircleSectorSize > 0)
        {
            const int growSectorBy
                = (params.config.minCircleSectorSize
                   - CircleSector::positive_mod(route->sector) + 1)
                  / 2;

            if (growSectorBy > 0)
            {
                route->sector.extend(route->sector.start - growSectorBy);
                route->sector.extend(route->sector.end + growSectorBy);
            }
        }
    }
}
