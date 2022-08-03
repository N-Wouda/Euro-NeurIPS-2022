#include "operators.h"

bool moveTwoClientsReversed(int &nbMoves,
                            bool &searchCompleted,
                            LocalSearch::Node *nodeU,
                            LocalSearch::Node *nodeV,
                            LocalSearch::Penalties const &penalties,
                            Params const &params)
{
    if (nodeU == nodeV->next || nodeU->next == nodeV || nodeU->next->isDepot)
        return false;

    int costSuppU = params.dist(nodeU->prev->cour, nodeU->next->next->cour)
                    - params.dist(nodeU->prev->cour, nodeU->cour)
                    - params.dist(nodeU->cour, nodeU->next->cour)
                    - params.dist(nodeU->next->cour, nodeU->next->next->cour);
    int costSuppV = params.dist(nodeV->cour, nodeU->next->cour)
                    + params.dist(nodeU->next->cour, nodeU->cour)
                    + params.dist(nodeU->cour, nodeV->next->cour)
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
                                                 nodeU->next->twData,
                                                 nodeU->twData,
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
            // - X - U - Y - ... - end
            auto const routeUTwData = LocalSearch::TimeWindowData::merge(
                nodeU->prev->prefixTwData,
                nodeU->next->next->mergeSegmentTwData(nodeV),
                nodeU->next->twData,
                nodeU->twData,
                nodeV->next->postfixTwData);

            costSuppU += penalties.timeWarp(routeUTwData);
        }
        else
        {
            // Edge case U directly after V is excluded from beginning of
            // function start - ... - V - X - U - Y - ... - UPrev - XNext - ...
            // - end
            auto const routeUTwData = LocalSearch::TimeWindowData::merge(
                nodeV->prefixTwData,
                nodeU->next->twData,
                nodeU->twData,
                nodeV->next->mergeSegmentTwData(nodeU->prev),
                nodeU->next->next->postfixTwData);

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
