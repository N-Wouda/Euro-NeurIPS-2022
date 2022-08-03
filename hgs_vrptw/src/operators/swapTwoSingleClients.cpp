#include "operators.h"

bool swapTwoSingleClients(int &nbMoves,
                          bool &searchCompleted,
                          LocalSearch::Node *nodeU,
                          LocalSearch::Node *nodeV,
                          LocalSearch::Penalties const &penalties,
                          Params const &params)
{
    if (nodeU->cour >= nodeV->cour)
        return false;

    if (nodeU->cour == nodeV->prev->cour || nodeU->cour == nodeV->next->cour)
        return false;

    int costSuppU = params.dist(nodeU->prev->cour, nodeV->cour)
                    + params.dist(nodeV->cour, nodeU->next->cour)
                    - params.dist(nodeU->prev->cour, nodeU->cour)
                    - params.dist(nodeU->cour, nodeU->next->cour);
    int costSuppV = params.dist(nodeV->prev->cour, nodeU->cour)
                    + params.dist(nodeU->cour, nodeV->next->cour)
                    - params.dist(nodeV->prev->cour, nodeV->cour)
                    - params.dist(nodeV->cour, nodeV->next->cour);

    if (nodeU->route != nodeV->route)
    {
        if (nodeU->route->load <= params.vehicleCapacity
            && nodeU->route->twData.timeWarp == 0
            && nodeV->route->load <= params.vehicleCapacity
            && nodeV->route->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData
            = LocalSearch::TimeWindowData::merge(nodeU->prev->prefixTwData,
                                                 nodeV->twData,
                                                 nodeU->next->postfixTwData);
        auto routeVTwData
            = LocalSearch::TimeWindowData::merge(nodeV->prev->prefixTwData,
                                                 nodeU->twData,
                                                 nodeV->next->postfixTwData);

        costSuppU += penalties.load(nodeU->route->load
                                    + params.clients[nodeV->cour].demand
                                    - params.clients[nodeU->cour].demand)
                     + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

        costSuppV += penalties.load(nodeV->route->load
                                    + params.clients[nodeU->cour].demand
                                    - params.clients[nodeV->cour].demand)
                     + penalties.timeWarp(routeVTwData) - nodeV->route->penalty;
    }
    else
    {
        if (nodeU->route->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Swap within the same route
        if (nodeU->position < nodeV->position)
        {
            // Edge case V directly after U, so X == V is excluded, V directly
            // after X so XNext == V works start - ... - UPrev - V - X - ... -
            // VPrev - U - Y - ... - end
            auto const routeUTwData = LocalSearch::TimeWindowData::merge(
                nodeU->prev->prefixTwData,
                nodeV->twData,
                nodeU->next->mergeSegmentTwData(nodeV->prev),
                nodeU->twData,
                nodeV->next->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - VPrev - U - Y - ... - UPrev - V - X - ...
            // - end
            auto const routeUTwData = LocalSearch::TimeWindowData::merge(
                nodeV->prev->prefixTwData,
                nodeU->twData,
                nodeV->next->mergeSegmentTwData(nodeU->prev),
                nodeV->twData,
                nodeU->next->postfixTwData);

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
    operators::updateRouteData(nodeU->route, nbMoves, penalties, params);
    if (nodeU->route != nodeV->route)
        operators::updateRouteData(nodeV->route, nbMoves, penalties, params);

    return true;
}
