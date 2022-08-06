#include "operators.h"

#include "TimeWindowSegment.h"

bool swapTwoSingleClients(Node *nodeU, Node *nodeV, Penalties const &penalties)
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
        if (nodeU->route->isFeasible() && nodeV->route->isFeasible()
            && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = TimeWindowSegment::merge(
            nodeU->prev->twBefore, nodeV->tw, nodeU->next->twAfter);
        auto routeVTwData = TimeWindowSegment::merge(
            nodeV->prev->twBefore, nodeU->tw, nodeV->next->twAfter);

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
        if (!nodeU->route->hasTimeWarp() && costSuppU + costSuppV >= 0)
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
                nodeU->route->twBetween(nodeU->next, nodeV->prev),
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
                nodeV->route->twBetween(nodeV->next, nodeU->prev),
                nodeV->tw,
                nodeU->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(nodeU->route->load) - nodeU->route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    nodeU->swapWith(nodeV);

    return true;
}
