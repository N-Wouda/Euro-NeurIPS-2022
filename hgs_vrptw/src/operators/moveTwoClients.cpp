#include "operators.h"

bool moveTwoClients(int &nbMoves,
                    bool &searchCompleted,
                    LocalSearch::Node *nodeU,
                    LocalSearch::Node *nodeV,
                    LocalSearch::Penalties const &penalties,
                    Params const &params)
{
    if (nodeU == nodeV->next || nodeV == nodeU->next || nodeU->next->isDepot)
        return false;

    int costSuppU = params.dist(nodeU->prev->cour, nodeU->next->next->cour)
                    - params.dist(nodeU->prev->cour, nodeU->cour)
                    - params.dist(nodeU->next->cour, nodeU->next->next->cour);
    int costSuppV = params.dist(nodeV->cour, nodeU->cour)
                    + params.dist(nodeU->next->cour, nodeV->next->cour)
                    - params.dist(nodeV->cour, nodeV->next->cour);

    if (nodeU->route != nodeV->route)
    {
        if (nodeU->route->load <= params.vehicleCapacity
            && nodeU->route->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        auto routeUTwData
            = nodeU->prev->prefixTwData.merge(nodeU->next->next->postfixTwData);
        auto routeVTwData
            = LocalSearch::TimeWindowData::merge(nodeV->prefixTwData,
                                                 nodeU->twData,
                                                 nodeU->next->twData,
                                                 nodeV->next->postfixTwData);

        costSuppU += penalties.load(nodeU->route->load
                                    - params.clients[nodeU->cour].demand
                                    - params.clients[nodeU->next->cour].demand)
                     + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

        costSuppV += penalties.load(nodeV->route->load
                                    + params.clients[nodeU->cour].demand
                                    + params.clients[nodeU->next->cour].demand)
                     + penalties.timeWarp(routeVTwData) - nodeV->route->penalty;
    }
    else
    {
        if (nodeU->route->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
        {
            return false;
        }

        // Move within the same route
        if (nodeU->position < nodeV->position)
        {
            // Edge case V directly after U, so X == V is excluded, V directly
            // after X so XNext == V works start - ... - UPrev - XNext - ... - V
            // - U - X - Y - ... - end
            auto const routeUTwData = LocalSearch::TimeWindowData::merge(
                nodeU->prev->prefixTwData,
                nodeU->next->next->mergeSegmentTwData(nodeV),
                nodeU->twData,
                nodeU->next->twData,
                nodeV->next->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - U - X - Y - ... - UPrev - XNext - ...
            // - end
            auto const routeUTwData = LocalSearch::TimeWindowData::merge(
                nodeV->prefixTwData,
                nodeU->twData,
                nodeU->next->twData,
                nodeV->next->mergeSegmentTwData(nodeU->prev),
                nodeU->next->next->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }

        // Compute new total penalty
        costSuppU += penalties.load(nodeU->route->load) - nodeU->route->penalty;
    }

    if (costSuppU + costSuppV >= 0)
        return false;

    auto *routeU = nodeU->route;

    operators::insertNode(nodeU->next, nodeV);  // insert X after V, and U after
    operators::insertNode(nodeU, nodeV);        // V, so we get V -> U -> X.

    nbMoves++;
    searchCompleted = false;
    operators::updateRouteData(routeU, nbMoves, penalties, params);
    if (routeU != nodeV->route)
        operators::updateRouteData(nodeV->route, nbMoves, penalties, params);

    return true;
}
