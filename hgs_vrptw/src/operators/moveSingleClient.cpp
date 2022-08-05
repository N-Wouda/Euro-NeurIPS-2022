#include "operators.h"

#include "TimeWindowSegment.h"

bool moveSingleClient(Node *nodeU,
                      Node *nodeV,
                      Penalties const &penalties)
{
    auto const &params = *nodeU->params;

    // If U already comes directly after V, this move has no effect
    if (nodeU->client == nodeV->next->client)
        return false;

    int costSuppU = params.dist(nodeU->prev->client, nodeU->next->client)
                    - params.dist(nodeU->prev->client, nodeU->client)
                    - params.dist(nodeU->client, nodeU->next->client);
    int costSuppV = params.dist(nodeV->client, nodeU->client)
                    + params.dist(nodeU->client, nodeV->next->client)
                    - params.dist(nodeV->client, nodeV->next->client);

    if (nodeU->route != nodeV->route)
    {
        if (!nodeU->route->hasExcessCapacity()
            && !nodeU->route->twData.hasTimeWarp()
            && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = TimeWindowSegment::merge(nodeU->prev->twBefore,
                                                     nodeU->next->twAfter);
        auto routeVTwData = TimeWindowSegment::merge(
            nodeV->twBefore, nodeU->tw, nodeV->next->twAfter);

        costSuppU += penalties.load(nodeU->route->load
                                    - params.clients[nodeU->client].demand)
                     + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

        costSuppV += penalties.load(nodeV->route->load
                                    + params.clients[nodeU->client].demand)
                     + penalties.timeWarp(routeVTwData) - nodeV->route->penalty;
    }
    else
    {
        if (!nodeU->route->twData.hasTimeWarp() && costSuppU + costSuppV >= 0)
            return false;

        // Move within the same route
        if (nodeU->position < nodeV->position)
        {
            // Edge case V directly after U, so X == V, this works
            // start - ... - UPrev - X - ... - V - U - Y - ... - end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeU->prev->twBefore,
                nodeU->next->mergeSegmentTwData(nodeV),
                nodeU->tw,
                nodeV->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - U - Y - ... - UPrev - X - ... - end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeV->twBefore,
                nodeU->tw,
                nodeV->next->mergeSegmentTwData(nodeU->prev),
                nodeU->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(nodeU->route->load) - nodeU->route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    operators::insertNode(nodeU, nodeV);

    return true;
}
