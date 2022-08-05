#include "operators.h"

#include "TimeWindowSegment.h"

bool twoOptBetweenTrips(Node *nodeU,
                        Node *nodeV,
                        Penalties const &penalties)
{
    auto const &params = *nodeU->params;

    if (nodeU->route->idx >= nodeV->route->idx)
        return false;

    int costSuppU = params.dist(nodeU->client, nodeV->next->client)
                    - params.dist(nodeU->client, nodeU->next->client);
    int costSuppV = params.dist(nodeV->client, nodeU->next->client)
                    - params.dist(nodeV->client, nodeV->next->client);

    if (!nodeU->route->hasExcessCapacity()
        && !nodeU->route->twData.hasTimeWarp()
        && !nodeV->route->hasExcessCapacity()
        && !nodeV->route->twData.hasTimeWarp() && costSuppU + costSuppV >= 0)
    {
        return false;
    }

    auto routeUTwData = TimeWindowSegment::merge(nodeU->twBefore,
                                              nodeV->next->twAfter);
    auto routeVTwData = TimeWindowSegment::merge(nodeV->twBefore,
                                              nodeU->next->twAfter);

    costSuppU += penalties.load(nodeU->cumulatedLoad + nodeV->route->load
                                - nodeV->cumulatedLoad)
                 + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

    costSuppV += penalties.load(nodeV->cumulatedLoad + nodeU->route->load
                                - nodeU->cumulatedLoad)
                 + penalties.timeWarp(routeVTwData) - nodeV->route->penalty;

    if (costSuppU + costSuppV >= 0)
        return false;

    auto *itRouteU = nodeU->next;
    auto *itRouteV = nodeV->next;

    auto *insertLocation = nodeU;
    while (!itRouteV->isDepot)
    {
        auto *current = itRouteV;
        itRouteV = itRouteV->next;
        operators::insertNode(current, insertLocation);
        insertLocation = current;
    }

    insertLocation = nodeV;
    while (!itRouteU->isDepot)
    {
        auto *current = itRouteU;
        itRouteU = itRouteU->next;
        operators::insertNode(current, insertLocation);
        insertLocation = current;
    }

    return true;
}
