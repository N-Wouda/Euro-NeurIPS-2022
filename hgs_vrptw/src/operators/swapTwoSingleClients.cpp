#include "operators.h"

#include "TimeWindowSegment.h"

bool swapTwoSingleClients(int &nbMoves,
                          bool &searchCompleted,
                          Node *nodeU,
                          Node *nodeV,
                          LocalSearch::Penalties const &penalties)
{
    auto const &params = *nodeU->params;

    if (nodeU->client >= nodeV->client)
        return false;

    if (nodeU->client == nodeV->prev->client
        || nodeU->client == nodeV->next->client)
        return false;

    int costSuppU = params.dist(nodeU->prev->client, nodeV->client)
                    + params.dist(nodeV->client, nodeU->next->client)
                    - params.dist(nodeU->prev->client, nodeU->client)
                    - params.dist(nodeU->client, nodeU->next->client);
    int costSuppV = params.dist(nodeV->prev->client, nodeU->client)
                    + params.dist(nodeU->client, nodeV->next->client)
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

        auto routeUTwData = TimeWindowSegment::merge(nodeU->prev->twBefore,
                                                  nodeV->tw,
                                                  nodeU->next->twAfter);
        auto routeVTwData = TimeWindowSegment::merge(nodeV->prev->twBefore,
                                                  nodeU->tw,
                                                  nodeV->next->twAfter);

        costSuppU += penalties.load(nodeU->route->load
                                    + params.clients[nodeV->client].demand
                                    - params.clients[nodeU->client].demand)
                     + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

        costSuppV += penalties.load(nodeV->route->load
                                    + params.clients[nodeU->client].demand
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
            // Edge case V directly after U, so X == V is excluded, V directly
            // after X so XNext == V works start - ... - UPrev - V - X - ... -
            // VPrev - U - Y - ... - end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeU->prev->twBefore,
                nodeV->tw,
                nodeU->next->mergeSegmentTwData(nodeV->prev),
                nodeU->tw,
                nodeV->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - VPrev - U - Y - ... - UPrev - V - X - ...
            // - end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeV->prev->twBefore,
                nodeU->tw,
                nodeV->next->mergeSegmentTwData(nodeU->prev),
                nodeV->tw,
                nodeU->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(nodeU->route->load) - nodeU->route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    operators::swapNode(nodeU, nodeV);
    nbMoves++;
    searchCompleted = false;
    operators::updateRouteData(nodeU->route, nbMoves, penalties);
    if (nodeU->route != nodeV->route)
        operators::updateRouteData(nodeV->route, nbMoves, penalties);

    return true;
}
