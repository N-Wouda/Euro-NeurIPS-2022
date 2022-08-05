#include "operators.h"

#include "TimeWindowSegment.h"

bool swapTwoClientsForOne(int &nbMoves,
                          bool &searchCompleted,
                          Node *nodeU,
                          Node *nodeV,
                          LocalSearch::Penalties const &penalties)
{
    auto const &params = *nodeU->params;

    if (nodeU == nodeV->prev || nodeU->next == nodeV->prev
        || nodeU == nodeV->next || nodeU->next->isDepot)
        return false;

    int costSuppU = params.dist(nodeU->prev->client, nodeV->client)
                    + params.dist(nodeV->client, nodeU->next->next->client)
                    - params.dist(nodeU->prev->client, nodeU->client)
                    - params.dist(nodeU->next->client, nodeU->next->next->client);
    int costSuppV = params.dist(nodeV->prev->client, nodeU->client)
                    + params.dist(nodeU->next->client, nodeV->next->client)
                    - params.dist(nodeV->prev->client, nodeV->client)
                    - params.dist(nodeV->client, nodeV->next->client);

    if (nodeU->route != nodeV->route)
    {
        if (nodeU->route->load <= params.vehicleCapacity
            && !nodeU->route->twData.hasTimeWarp()
            && nodeV->route->load <= params.vehicleCapacity
            && !nodeV->route->twData.hasTimeWarp()
            && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData
            = TimeWindowSegment::merge(nodeU->prev->twBefore,
                                    nodeV->tw,
                                    nodeU->next->next->twAfter);
        auto routeVTwData = TimeWindowSegment::merge(nodeV->prev->twBefore,
                                                  nodeU->tw,
                                                  nodeU->next->tw,
                                                  nodeV->next->twAfter);

        costSuppU += penalties.load(nodeU->route->load
                                    + params.clients[nodeV->client].demand
                                    - params.clients[nodeU->client].demand
                                    - params.clients[nodeU->next->client].demand)
                     + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

        costSuppV += penalties.load(nodeV->route->load
                                    + params.clients[nodeU->client].demand
                                    + params.clients[nodeU->next->client].demand
                                    - params.clients[nodeV->client].demand)
                     + penalties.timeWarp(routeVTwData) - nodeV->route->penalty;
    }
    else
    {
        if (!nodeU->route->twData.hasTimeWarp() && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Swap within the same route
        if (nodeU->position < nodeV->position)
        {
            // start - ... - UPrev - V - XNext - ... - VPrev - U - X - Y - ... -
            // end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeU->prev->twBefore,
                nodeV->tw,
                nodeU->next->next->mergeSegmentTwData(nodeV->prev),
                nodeU->tw,
                nodeU->next->tw,
                nodeV->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // start - ... - VPrev - U - X - Y - ... - UPrev - V - XNext - ... -
            // end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeV->prev->twBefore,
                nodeU->tw,
                nodeU->next->tw,
                nodeV->next->mergeSegmentTwData(nodeU->prev),
                nodeV->tw,
                nodeU->next->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(nodeU->route->load) - nodeU->route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    operators::insertNode(nodeU->next, nodeV);
    operators::swapNode(nodeU, nodeV);

    nbMoves++;
    searchCompleted = false;
    operators::updateRouteData(nodeU->route, nbMoves, penalties);
    if (nodeU->route != nodeV->route)
        operators::updateRouteData(nodeV->route, nbMoves, penalties);

    return true;
}
