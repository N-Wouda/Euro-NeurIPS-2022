#include "operators.h"

#include "TimeWindowSegment.h"

bool swapTwoClientPairs(Node *nodeU, Node *nodeV, Penalties const &penalties)
{
    auto const &params = *nodeU->params;

    if (nodeU->client >= nodeV->client)
        return false;

    if (nodeU->next->isDepot || nodeV->next->isDepot
        || nodeV->next == nodeU->prev || nodeU == nodeV->next
        || nodeU->next == nodeV || nodeV == nodeU->next->next)
        return false;

    int costSuppU
        = params.dist(nodeU->prev->client, nodeV->client)
          + params.dist(nodeV->next->client, nodeU->next->next->client)
          - params.dist(nodeU->prev->client, nodeU->client)
          - params.dist(nodeU->next->client, nodeU->next->next->client);
    int costSuppV
        = params.dist(nodeV->prev->client, nodeU->client)
          + params.dist(nodeU->next->client, nodeV->next->next->client)
          - params.dist(nodeV->prev->client, nodeV->client)
          - params.dist(nodeV->next->client, nodeV->next->next->client);

    if (nodeU->route != nodeV->route)
    {
        if (nodeU->route->isFeasible() && nodeV->route->isFeasible()
            && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData
            = TimeWindowSegment::merge(nodeU->prev->twBefore,
                                       nodeV->tw,
                                       nodeV->next->tw,
                                       nodeU->next->next->twAfter);
        auto routeVTwData
            = TimeWindowSegment::merge(nodeV->prev->twBefore,
                                       nodeU->tw,
                                       nodeU->next->tw,
                                       nodeV->next->next->twAfter);

        costSuppU
            += penalties.load(nodeU->route->load
                              + params.clients[nodeV->client].demand
                              + params.clients[nodeV->next->client].demand
                              - params.clients[nodeU->client].demand
                              - params.clients[nodeU->next->client].demand)
               + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

        costSuppV
            += penalties.load(nodeV->route->load
                              + params.clients[nodeU->client].demand
                              + params.clients[nodeU->next->client].demand
                              - params.clients[nodeV->client].demand
                              - params.clients[nodeV->next->client].demand)
               + penalties.timeWarp(routeVTwData) - nodeV->route->penalty;
    }
    else
    {
        if (!nodeU->route->hasTimeWarp() && costSuppU + costSuppV >= 0)
            return false;

        // Swap within the same route
        if (nodeU->position < nodeV->position)
        {
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeU->prev->twBefore,
                nodeV->tw,
                nodeV->next->tw,
                nodeU->route->twBetween(nodeU->next->next, nodeV->prev),
                nodeU->tw,
                nodeU->next->tw,
                nodeV->next->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeV->prev->twBefore,
                nodeU->tw,
                nodeU->next->tw,
                nodeV->route->twBetween(nodeV->next->next, nodeU->prev),
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

    nodeU->next->swapWith(nodeV->next);
    nodeU->swapWith(nodeV);

    return true;
}
