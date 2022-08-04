#include "operators.h"

#include "TimeWindowSegment.h"

bool swapTwoClientsForOne(int &nbMoves,
                          bool &searchCompleted,
                          LocalSearch::Node *nodeU,
                          LocalSearch::Node *nodeV,
                          LocalSearch::Penalties const &penalties,
                          Params const &params)
{
    if (nodeU == nodeV->prev || nodeU->next == nodeV->prev
        || nodeU == nodeV->next || nodeU->next->isDepot)
        return false;

    int costSuppU = params.dist(nodeU->prev->cour, nodeV->cour)
                    + params.dist(nodeV->cour, nodeU->next->next->cour)
                    - params.dist(nodeU->prev->cour, nodeU->cour)
                    - params.dist(nodeU->next->cour, nodeU->next->next->cour);
    int costSuppV = params.dist(nodeV->prev->cour, nodeU->cour)
                    + params.dist(nodeU->next->cour, nodeV->next->cour)
                    - params.dist(nodeV->prev->cour, nodeV->cour)
                    - params.dist(nodeV->cour, nodeV->next->cour);

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
            = TimeWindowSegment::merge(nodeU->prev->prefixTwData,
                                    nodeV->twData,
                                    nodeU->next->next->postfixTwData);
        auto routeVTwData = TimeWindowSegment::merge(nodeV->prev->prefixTwData,
                                                  nodeU->twData,
                                                  nodeU->next->twData,
                                                  nodeV->next->postfixTwData);

        costSuppU += penalties.load(nodeU->route->load
                                    + params.clients[nodeV->cour].demand
                                    - params.clients[nodeU->cour].demand
                                    - params.clients[nodeU->next->cour].demand)
                     + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

        costSuppV += penalties.load(nodeV->route->load
                                    + params.clients[nodeU->cour].demand
                                    + params.clients[nodeU->next->cour].demand
                                    - params.clients[nodeV->cour].demand)
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
                nodeU->prev->prefixTwData,
                nodeV->twData,
                nodeU->next->next->mergeSegmentTwData(nodeV->prev),
                nodeU->twData,
                nodeU->next->twData,
                nodeV->next->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // start - ... - VPrev - U - X - Y - ... - UPrev - V - XNext - ... -
            // end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeV->prev->prefixTwData,
                nodeU->twData,
                nodeU->next->twData,
                nodeV->next->mergeSegmentTwData(nodeU->prev),
                nodeV->twData,
                nodeU->next->next->postfixTwData);

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
    operators::updateRouteData(nodeU->route, nbMoves, penalties, params);
    if (nodeU->route != nodeV->route)
        operators::updateRouteData(nodeV->route, nbMoves, penalties, params);

    return true;
}
