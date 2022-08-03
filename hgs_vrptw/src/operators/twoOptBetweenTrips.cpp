#include "operators.h"

bool twoOptBetweenTrips(int &nbMoves,
                        bool &searchCompleted,
                        LocalSearch::Node *nodeU,
                        LocalSearch::Node *nodeV,
                        LocalSearch::Penalties const &penalties,
                        Params const &params)
{
    if (nodeU->route->cour >= nodeV->route->cour)
        return false;

    int costSuppU = params.dist(nodeU->cour, nodeV->next->cour)
                    - params.dist(nodeU->cour, nodeU->next->cour);
    int costSuppV = params.dist(nodeV->cour, nodeU->next->cour)
                    - params.dist(nodeV->cour, nodeV->next->cour);

    if (nodeU->route->load <= params.vehicleCapacity
        && nodeU->route->twData.timeWarp == 0
        && nodeV->route->load <= params.vehicleCapacity
        && nodeV->route->twData.timeWarp == 0 && costSuppU + costSuppV >= 0)
    {
        return false;
    }

    auto routeUTwData = nodeU->prefixTwData.merge(nodeV->next->postfixTwData);
    auto routeVTwData = nodeV->prefixTwData.merge(nodeU->next->postfixTwData);

    costSuppU += penalties.load(nodeU->cumulatedLoad + nodeV->route->load
                                - nodeV->cumulatedLoad)
                 + penalties.timeWarp(routeUTwData) - nodeU->route->penalty;

    costSuppV += penalties.load(nodeV->cumulatedLoad + nodeU->route->load
                                - nodeU->cumulatedLoad)
                 + penalties.timeWarp(routeVTwData) - nodeV->route->penalty;

    if (costSuppU + costSuppV >= 0)
        return false;

    auto *routeU = nodeU->route;  // these values change in the following, so
    auto *routeV = nodeV->route;  // must be set here already.
    auto *itRouteU = nodeU->next;
    auto *itRouteV = nodeV->next;

    auto *insertLocation = nodeU;
    while (!itRouteV->isDepot)
    {
        auto *current = itRouteV;
        itRouteV = itRouteV->next;
        operators::insertNode(current, insertLocation);
        insertLocation = current;
    }

    insertLocation = nodeV;
    while (!itRouteU->isDepot)
    {
        auto *current = itRouteU;
        itRouteU = itRouteU->next;
        operators::insertNode(current, insertLocation);
        insertLocation = current;
    }

    nbMoves++;
    searchCompleted = false;
    operators::updateRouteData(routeU, nbMoves, penalties, params);
    operators::updateRouteData(routeV, nbMoves, penalties, params);

    return true;
}
