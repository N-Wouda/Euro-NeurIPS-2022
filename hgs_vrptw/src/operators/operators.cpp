#include "operators.h"

#include "TimeWindowSegment.h"

int operators::singleMoveCost(Node *nodeU,
                              Node *nodeV,
                              Penalties const &penalties)
{
    auto const &params = *nodeU->params;

    // If U already comes directly after V, this move has no effect
    if (nodeU->client == nodeV->next->client)
        return 0;

    int costSuppU = params.dist(nodeU->prev->client, nodeU->next->client)
                    - params.dist(nodeU->prev->client, nodeU->client)
                    - params.dist(nodeU->client, nodeU->next->client);
    int costSuppV = params.dist(nodeV->client, nodeU->client)
                    + params.dist(nodeU->client, nodeV->next->client)
                    - params.dist(nodeV->client, nodeV->next->client);

    if (nodeU->route != nodeV->route)
    {
        if (nodeU->route->isFeasible() && costSuppU + costSuppV >= 0)
            return costSuppU + costSuppV;

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
        if (!nodeU->route->hasTimeWarp() && costSuppU + costSuppV >= 0)
            return costSuppU + costSuppV;

        // Move within the same route
        if (nodeU->position < nodeV->position)
        {
            // Edge case V directly after U, so X == V, this works
            // start - ... - UPrev - X - ... - V - U - Y - ... - end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeU->prev->twBefore,
                nodeU->route->twBetween(nodeU->next, nodeV),
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
                nodeV->route->twBetween(nodeV->next, nodeU->prev),
                nodeU->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(nodeU->route->load) - nodeU->route->penalty;
    }

    return costSuppU + costSuppV;
}

int operators::twoSwapCost(Node *nodeU, Node *nodeV, Penalties const &penalties)
{
    auto const &params = *nodeU->params;

    if (nodeU == nodeV->prev || nodeU->next == nodeV->prev
        || nodeU == nodeV->next || nodeU->next->isDepot)
        return 0;

    int costSuppU
        = params.dist(nodeU->prev->client, nodeV->client)
          + params.dist(nodeV->client, nodeU->next->next->client)
          - params.dist(nodeU->prev->client, nodeU->client)
          - params.dist(nodeU->next->client, nodeU->next->next->client);
    int costSuppV = params.dist(nodeV->prev->client, nodeU->client)
                    + params.dist(nodeU->next->client, nodeV->next->client)
                    - params.dist(nodeV->prev->client, nodeV->client)
                    - params.dist(nodeV->client, nodeV->next->client);

    if (nodeU->route != nodeV->route)
    {
        if (nodeU->route->isFeasible() && nodeV->route->isFeasible()
            && costSuppU + costSuppV >= 0)
        {
            return costSuppU + costSuppV;
        }

        auto routeUTwData = TimeWindowSegment::merge(
            nodeU->prev->twBefore, nodeV->tw, nodeU->next->next->twAfter);
        auto routeVTwData = TimeWindowSegment::merge(nodeV->prev->twBefore,
                                                     nodeU->tw,
                                                     nodeU->next->tw,
                                                     nodeV->next->twAfter);

        costSuppU
            += penalties.load(nodeU->route->load
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
        if (!nodeU->route->hasTimeWarp() && costSuppU + costSuppV >= 0)
            return costSuppU + costSuppV;

        // Swap within the same route
        if (nodeU->position < nodeV->position)
        {
            // start - ... - UPrev - V - XNext - ... - VPrev - U - X - Y - ... -
            // end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeU->prev->twBefore,
                nodeV->tw,
                nodeU->route->twBetween(nodeU->next->next, nodeV->prev),
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
                nodeV->route->twBetween(nodeV->next, nodeU->prev),
                nodeV->tw,
                nodeU->next->next->twAfter);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(nodeU->route->load) - nodeU->route->penalty;
    }

    return costSuppU + costSuppV;
}
