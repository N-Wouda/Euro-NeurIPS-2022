#include "operators.h"

#include "TimeWindowSegment.h"

bool swapTwoClientPairs(int &nbMoves,
                        bool &searchCompleted,
                        LocalSearch::Node *nodeU,
                        LocalSearch::Node *nodeV,
                        LocalSearch::Penalties const &penalties,
                        Params const &params)
{
    if (nodeU->cour >= nodeV->cour)
        return false;

    if (nodeU->next->isDepot || nodeV->next->isDepot
        || nodeV->next == nodeU->prev || nodeU == nodeV->next
        || nodeU->next == nodeV || nodeV == nodeU->next->next)
        return false;

    int costSuppU = params.dist(nodeU->prev->cour, nodeV->cour)
                    + params.dist(nodeV->next->cour, nodeU->next->next->cour)
                    - params.dist(nodeU->prev->cour, nodeU->cour)
                    - params.dist(nodeU->next->cour, nodeU->next->next->cour);
    int costSuppV = params.dist(nodeV->prev->cour, nodeU->cour)
                    + params.dist(nodeU->next->cour, nodeV->next->next->cour)
                    - params.dist(nodeV->prev->cour, nodeV->cour)
                    - params.dist(nodeV->next->cour, nodeV->next->next->cour);

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
            nodeU->prev->prefixTwData,
            nodeV->twData,
            nodeV->next->twData,
            nodeU->next->next->postfixTwData);
        auto routeVTwData = TimeWindowSegment::merge(
            nodeV->prev->prefixTwData,
            nodeU->twData,
            nodeU->next->twData,
            nodeV->next->next->postfixTwData);

        costSuppU += penalties.load(nodeU->route->load
                                    + params.clients[nodeV->cour].demand
                                    + params.clients[nodeV->next->cour].demand
                                    - params.clients[nodeU->cour].demand
                                    - params.clients[nodeU->next->cour].demand)
                     + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

        costSuppV += penalties.load(nodeV->route->load
                                    + params.clients[nodeU->cour].demand
                                    + params.clients[nodeU->next->cour].demand
                                    - params.clients[nodeV->cour].demand
                                    - params.clients[nodeV->next->cour].demand)
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
                nodeU->prev->prefixTwData,
                nodeV->twData,
                nodeV->next->twData,
                nodeU->next->next->mergeSegmentTwData(nodeV->prev),
                nodeU->twData,
                nodeU->next->twData,
                nodeV->next->next->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // start - ... - VPrev - U - X - YNext - ... - UPrev - V - Y - XNext
            // - ... - end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeV->prev->prefixTwData,
                nodeU->twData,
                nodeU->next->twData,
                nodeV->next->next->mergeSegmentTwData(nodeU->prev),
                nodeV->twData,
                nodeV->next->twData,
                nodeU->next->next->postfixTwData);

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
