#include "operators.h"

#include "TimeWindowSegment.h"

bool moveTwoClientsReversed(int &nbMoves,
                            bool &searchCompleted,
                            Node *nodeU,
                            Node *nodeV,
                            LocalSearch::Penalties const &penalties,
                            Params const &params)
{
    if (nodeU == nodeV->next || nodeU->next == nodeV || nodeU->next->isDepot)
        return false;

    int costSuppU = params.dist(nodeU->prev->client, nodeU->next->next->client)
                    - params.dist(nodeU->prev->client, nodeU->client)
                    - params.dist(nodeU->client, nodeU->next->client)
                    - params.dist(nodeU->next->client, nodeU->next->next->client);
    int costSuppV = params.dist(nodeV->client, nodeU->next->client)
                    + params.dist(nodeU->next->client, nodeU->client)
                    + params.dist(nodeU->client, nodeV->next->client)
                    - params.dist(nodeV->client, nodeV->next->client);

    if (nodeU->route != nodeV->route)
    {
        if (nodeU->route->load <= params.vehicleCapacity
            && !nodeU->route->twData.hasTimeWarp()
            && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = TimeWindowSegment::merge(
            nodeU->prev->twBefore, nodeU->next->next->twAfter);
        auto routeVTwData = TimeWindowSegment::merge(nodeV->twBefore,
                                                  nodeU->next->tw,
                                                  nodeU->tw,
                                                  nodeV->next->twAfter);

        costSuppU += penalties.load(nodeU->route->load
                                    - params.clients[nodeU->client].demand
                                    - params.clients[nodeU->next->client].demand)
                     + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

        costSuppV += penalties.load(nodeV->route->load
                                    + params.clients[nodeU->client].demand
                                    + params.clients[nodeU->next->client].demand)
                     + penalties.timeWarp(routeVTwData) - nodeV->route->penalty;
    }
    else
    {
        if (!nodeU->route->twData.hasTimeWarp() && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Move within the same route
        if (nodeU->position < nodeV->position)
        {
            // Edge case V directly after U, so X == V is excluded, V directly
            // after X so XNext == V works start - ... - UPrev - XNext - ... - V
            // - X - U - Y - ... - end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeU->prev->twBefore,
                nodeU->next->next->mergeSegmentTwData(nodeV),
                nodeU->next->tw,
                nodeU->tw,
                nodeV->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - X - U - Y - ... - UPrev - XNext - ...
            // - end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeV->twBefore,
                nodeU->next->tw,
                nodeU->tw,
                nodeV->next->mergeSegmentTwData(nodeU->prev),
                nodeU->next->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(nodeU->route->load) - nodeU->route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    auto *nodeX = nodeU->next;
    auto *routeU = nodeU->route;

    operators::insertNode(nodeU, nodeV);  // insert U after V
    operators::insertNode(nodeX, nodeV);  // insert X after V (so V -> X -> U)

    nbMoves++;
    searchCompleted = false;
    operators::updateRouteData(routeU, nbMoves, penalties, params);
    if (routeU != nodeV->route)
        operators::updateRouteData(nodeV->route, nbMoves, penalties, params);

    return true;
}
