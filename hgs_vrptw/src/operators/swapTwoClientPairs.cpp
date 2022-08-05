#include "operators.h"

#include "TimeWindowSegment.h"

bool swapTwoClientPairs(int &nbMoves,
                        bool &searchCompleted,
                        Node *nodeU,
                        Node *nodeV,
                        LocalSearch::Penalties const &penalties,
                        Params const &params)
{
    if (nodeU->client >= nodeV->client)
        return false;

    if (nodeU->next->isDepot || nodeV->next->isDepot
        || nodeV->next == nodeU->prev || nodeU == nodeV->next
        || nodeU->next == nodeV || nodeV == nodeU->next->next)
        return false;

    int costSuppU = params.dist(nodeU->prev->client, nodeV->client)
                    + params.dist(nodeV->next->client, nodeU->next->next->client)
                    - params.dist(nodeU->prev->client, nodeU->client)
                    - params.dist(nodeU->next->client, nodeU->next->next->client);
    int costSuppV = params.dist(nodeV->prev->client, nodeU->client)
                    + params.dist(nodeU->next->client, nodeV->next->next->client)
                    - params.dist(nodeV->prev->client, nodeV->client)
                    - params.dist(nodeV->next->client, nodeV->next->next->client);

    if (nodeU->route != nodeV->route)
    {
        if (nodeU->route->load <= params.vehicleCapacity
            && !nodeU->route->twData.hasTimeWarp()
            && nodeV->route->load <= params.vehicleCapacity
            && !nodeV->route->twData.hasTimeWarp() && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = TimeWindowSegment::merge(
            nodeU->prev->twBefore,
            nodeV->tw,
            nodeV->next->tw,
            nodeU->next->next->twAfter);
        auto routeVTwData = TimeWindowSegment::merge(
            nodeV->prev->twBefore,
            nodeU->tw,
            nodeU->next->tw,
            nodeV->next->next->twAfter);

        costSuppU += penalties.load(nodeU->route->load
                                    + params.clients[nodeV->client].demand
                                    + params.clients[nodeV->next->client].demand
                                    - params.clients[nodeU->client].demand
                                    - params.clients[nodeU->next->client].demand)
                     + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

        costSuppV += penalties.load(nodeV->route->load
                                    + params.clients[nodeU->client].demand
                                    + params.clients[nodeU->next->client].demand
                                    - params.clients[nodeV->client].demand
                                    - params.clients[nodeV->next->client].demand)
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
            // start - ... - UPrev - V - Y - XNext - ... - VPrev - U - X - YNext
            // - ... - end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeU->prev->twBefore,
                nodeV->tw,
                nodeV->next->tw,
                nodeU->next->next->mergeSegmentTwData(nodeV->prev),
                nodeU->tw,
                nodeU->next->tw,
                nodeV->next->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // start - ... - VPrev - U - X - YNext - ... - UPrev - V - Y - XNext
            // - ... - end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeV->prev->twBefore,
                nodeU->tw,
                nodeU->next->tw,
                nodeV->next->next->mergeSegmentTwData(nodeU->prev),
                nodeV->tw,
                nodeV->next->tw,
                nodeU->next->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(nodeU->route->load) - nodeU->route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    operators::swapNode(nodeU->next, nodeV->next);
    operators::swapNode(nodeU, nodeV);

    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;

    operators::updateRouteData(nodeU->route, nbMoves, penalties, params);
    if (nodeU->route != nodeV->route)
        operators::updateRouteData(nodeV->route, nbMoves, penalties, params);

    return true;
}
