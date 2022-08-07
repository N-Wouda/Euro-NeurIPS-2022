#include "operators.h"

#include "TimeWindowSegment.h"

bool twoOptWithinTrip(Node *nodeU, Node *nodeV, Penalties const &penalties)
{
    auto const &params = *nodeU->params;

    if (nodeU->route != nodeV->route)
        return false;

    if (nodeU->position + 1 >= nodeV->position)
        return false;

    int deltaCost = params.dist(nodeU->client, nodeV->client)
                    + params.dist(nodeU->next->client, nodeV->next->client)
                    - params.dist(nodeU->client, nodeU->next->client)
                    - params.dist(nodeV->client, nodeV->next->client)
                    + nodeV->cumulatedReversalDistance
                    - nodeU->next->cumulatedReversalDistance;

    if (!nodeU->route->hasTimeWarp() && deltaCost >= 0)
        return false;

    auto routeTwData = nodeU->twBefore;
    auto *itRoute = nodeV;
    while (itRoute != nodeU)
    {
        routeTwData = TimeWindowSegment::merge(routeTwData, itRoute->tw);
        itRoute = itRoute->prev;
    }
    routeTwData = TimeWindowSegment::merge(routeTwData, nodeV->next->twAfter);

    deltaCost += penalties.load(nodeU->route->load)
                 + penalties.timeWarp(routeTwData) - nodeU->route->penalty;

    if (deltaCost >= 0)
        return false;

    itRoute = nodeV;
    auto *insertionPoint = nodeU;
    auto *currNext = nodeU->next;

    while (itRoute != currNext)  // No need to move x, we pivot around it
    {
        auto *current = itRoute;
        itRoute = itRoute->prev;
        current->insertAfter(insertionPoint);
        insertionPoint = current;
    }

    return true;
}
