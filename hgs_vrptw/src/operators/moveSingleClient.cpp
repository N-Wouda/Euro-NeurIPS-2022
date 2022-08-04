#include "operators.h"

#include "TimeWindowSegment.h"

bool moveSingleClient(int &nbMoves,
                      bool &searchCompleted,
                      LocalSearch::Node *nodeU,
                      LocalSearch::Node *nodeV,
                      LocalSearch::Penalties const &penalties,
                      Params const &params)
{
    // If U already comes directly after V, this move has no effect
    if (nodeU->cour == nodeV->next->cour)
        return false;

    int costSuppU = params.dist(nodeU->prev->cour, nodeU->next->cour)
                    - params.dist(nodeU->prev->cour, nodeU->cour)
                    - params.dist(nodeU->cour, nodeU->next->cour);
    int costSuppV = params.dist(nodeV->cour, nodeU->cour)
                    + params.dist(nodeU->cour, nodeV->next->cour)
                    - params.dist(nodeV->cour, nodeV->next->cour);

    if (nodeU->route != nodeV->route)
    {
        if (nodeU->route->load <= params.vehicleCapacity
            && !nodeU->route->twData.hasTimeWarp()
            && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData = TimeWindowSegment::merge(nodeU->prev->prefixTwData,
                                                  nodeU->next->postfixTwData);
        auto routeVTwData = TimeWindowSegment::merge(
            nodeV->prefixTwData, nodeU->twData, nodeV->next->postfixTwData);

        costSuppU += penalties.load(nodeU->route->load
                                    - params.clients[nodeU->cour].demand)
                     + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

        costSuppV += penalties.load(nodeV->route->load
                                    + params.clients[nodeU->cour].demand)
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
            // Edge case V directly after U, so X == V, this works
            // start - ... - UPrev - X - ... - V - U - Y - ... - end
            auto const routeUTwData
                = TimeWindowSegment::merge(nodeU->prev->prefixTwData,
                                        nodeU->next->mergeSegmentTwData(nodeV),
                                        nodeU->twData,
                                        nodeV->next->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - U - Y - ... - UPrev - X - ... - end
            auto const routeUTwData = TimeWindowSegment::merge(
                nodeV->prefixTwData,
                nodeU->twData,
                nodeV->next->mergeSegmentTwData(nodeU->prev),
                nodeU->next->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(nodeU->route->load) - nodeU->route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    auto *routeU = nodeU->route;

    operators::insertNode(nodeU, nodeV);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    operators::updateRouteData(routeU, nbMoves, penalties, params);
    if (routeU != nodeV->route)
        operators::updateRouteData(nodeV->route, nbMoves, penalties, params);

    return true;
}
