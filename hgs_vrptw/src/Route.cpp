#include "Route.h"

#include <cmath>

void Route::update(int nbMoves, Penalties const &penalties)
{
    load = 0;

    int place = 0;
    int reverseDistance = 0;
    int cumulatedX = 0;
    int cumulatedY = 0;

    auto *node = this->depot;
    node->position = 0;
    node->cumulatedLoad = 0;
    node->cumulatedReversalDistance = 0;

    bool firstIt = true;
    auto seedTwD = node->tw;
    Node *seedNode = nullptr;

    while (!node->isDepot || firstIt)
    {
        node = node->next;
        place++;
        node->position = place;
        load += params->clients[node->client].demand;
        reverseDistance += params->dist(node->client, node->prev->client)
                           - params->dist(node->prev->client, node->client);
        node->cumulatedLoad = load;
        node->cumulatedReversalDistance = reverseDistance;
        node->twBefore
            = TimeWindowSegment::merge(node->prev->twBefore, node->tw);
        node->isSeed = false;
        node->nextSeed = nullptr;
        if (!node->isDepot)
        {
            cumulatedX += params->clients[node->client].x;
            cumulatedY += params->clients[node->client].y;
            if (firstIt)
                sector.initialize(params->clients[node->client].angle);
            else
                sector.extend(params->clients[node->client].angle);

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

    twData = node->twBefore;
    penalty = penalties.load(load) + penalties.timeWarp(this->twData);
    nbCustomers = place - 1;

    // Remember "when" this route has been last modified (will be used to filter
    // unnecessary move evaluations)
    whenLastModified = nbMoves;

    // Time window data in reverse direction, node should be end depot now
    do
    {
        node = node->prev;
        node->twAfter = TimeWindowSegment::merge(node->tw, node->next->twAfter);
    } while (!node->isDepot);

    if (this->nbCustomers == 0)
    {
        this->polarAngleBarycenter = 1.e30;
    }
    else
    {
        this->polarAngleBarycenter
            = atan2(cumulatedY / static_cast<double>(nbCustomers)
                        - params->clients[0].y,
                    cumulatedX / static_cast<double>(nbCustomers)
                        - params->clients[0].x);

        // Enforce minimum size of circle sector
        if (params->config.minCircleSectorSize > 0)
        {
            const int growSectorBy = (params->config.minCircleSectorSize
                                      - CircleSector::positive_mod(sector) + 1)
                                     / 2;

            if (growSectorBy > 0)
            {
                sector.extend(sector.start - growSectorBy);
                sector.extend(sector.end + growSectorBy);
            }
        }
    }
}
