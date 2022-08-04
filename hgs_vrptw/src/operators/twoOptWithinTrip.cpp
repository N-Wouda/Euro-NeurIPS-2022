#include "operators.h"

#include "TimeWindowSegment.h"

bool twoOptWithinTrip(int &nbMoves,
                      bool &searchCompleted,
                      LocalSearch::Node *nodeU,
                      LocalSearch::Node *nodeV,
                      LocalSearch::Penalties const &penalties,
                      Params const &params)
{
    if (nodeU->route != nodeV->route)
        return false;

    if (nodeU->position >= nodeV->position - 1)
        return false;

    int cost = params.dist(nodeU->cour, nodeV->cour)
               + params.dist(nodeU->next->cour, nodeV->next->cour)
               - params.dist(nodeU->cour, nodeU->next->cour)
               - params.dist(nodeV->cour, nodeV->next->cour)
               + nodeV->cumulatedReversalDistance
               - nodeU->next->cumulatedReversalDistance;

    if (!nodeU->route->twData.hasTimeWarp() && cost >= 0)
        return false;

    auto routeTwData = nodeU->prefixTwData;
    auto *itRoute = nodeV;
    while (itRoute != nodeU)
    {
        routeTwData = TimeWindowSegment::merge(routeTwData, itRoute->twData);
        itRoute = itRoute->prev;
    }
    routeTwData
        = TimeWindowSegment::merge(routeTwData, nodeV->next->postfixTwData);

    // Compute new total penalty
    cost += penalties.load(nodeU->route->load) + penalties.timeWarp(routeTwData)
            - nodeU->route->penalty;

    if (cost >= 0)
        return false;

    itRoute = nodeV;
    auto *insertionPoint = nodeU;
    auto *currNext = nodeU->next;

    while (itRoute != currNext)  // No need to move x, we pivot around it
    {
        auto *current = itRoute;
        itRoute = itRoute->prev;
        operators::insertNode(current, insertionPoint);
        insertionPoint = current;
    }

    nbMoves++;
    searchCompleted = false;
    operators::updateRouteData(nodeU->route, nbMoves, penalties, params);

    return true;
}
