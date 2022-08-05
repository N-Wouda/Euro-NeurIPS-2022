#include "operators.h"

#include "TimeWindowSegment.h"

bool twoOptBetweenTrips(int &nbMoves,
                        bool &searchCompleted,
                        Node *nodeU,
                        Node *nodeV,
                        LocalSearch::Penalties const &penalties,
                        Params const &params)
{
    if (nodeU->route->idx >= nodeV->route->idx)
        return false;

    int costSuppU = params.dist(nodeU->client, nodeV->next->client)
                    - params.dist(nodeU->client, nodeU->next->client);
    int costSuppV = params.dist(nodeV->client, nodeU->next->client)
                    - params.dist(nodeV->client, nodeV->next->client);

    if (nodeU->route->load <= params.vehicleCapacity
        && !nodeU->route->twData.hasTimeWarp()
        && nodeV->route->load <= params.vehicleCapacity
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

    auto *routeU = nodeU->route;  // these values change in the following, so
    auto *routeV = nodeV->route;  // must be set here already.
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

    nbMoves++;
    searchCompleted = false;
    operators::updateRouteData(routeU, nbMoves, penalties, params);
    operators::updateRouteData(routeV, nbMoves, penalties, params);

    return true;
}
