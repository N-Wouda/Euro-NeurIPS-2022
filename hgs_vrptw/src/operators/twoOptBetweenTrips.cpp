#include "operators.h"

#include "TimeWindowSegment.h"

bool twoOptBetweenTrips(Node *nodeU, Node *nodeV, Penalties const &penalties)
{
    using TWS = TimeWindowSegment;

    auto const &params = *nodeU->params;

    if (nodeU->route->idx >= nodeV->route->idx)
        return false;

    int costSuppU = params.dist(nodeU->client, nodeV->next->client)
                    - params.dist(nodeU->client, nodeU->next->client);
    int costSuppV = params.dist(nodeV->client, nodeU->next->client)
                    - params.dist(nodeV->client, nodeV->next->client);

    if (nodeU->route->isFeasible() && nodeV->route->isFeasible()
        && costSuppU + costSuppV >= 0)
    {
        return false;
    }

    auto const routeUTwData = TWS::merge(nodeU->twBefore, nodeV->next->twAfter);
    auto const routeVTwData = TWS::merge(nodeV->twBefore, nodeU->next->twAfter);

    int const deltaLoad = nodeU->cumulatedLoad - nodeV->cumulatedLoad;
    costSuppU += penalties.load(nodeV->route->load + deltaLoad)
                 + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

    costSuppV += penalties.load(nodeU->route->load - deltaLoad)
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
        current->insertAfter(insertLocation);
        insertLocation = current;
    }

    insertLocation = nodeV;
    while (!itRouteU->isDepot)
    {
        auto *current = itRouteU;
        itRouteU = itRouteU->next;
        current->insertAfter(insertLocation);
        insertLocation = current;
    }

    return true;
}
