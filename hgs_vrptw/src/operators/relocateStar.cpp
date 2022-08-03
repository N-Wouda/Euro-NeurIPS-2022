#include "operators.h"

bool relocateStar(int &nbMoves,
                  bool &searchCompleted,
                  LocalSearch::Route *routeU,
                  LocalSearch::Route *routeV,
                  LocalSearch::Penalties const &penalties,
                  Params const &params)
{
    int bestCost = 0;
    LocalSearch::Node *insertionPoint = nullptr;
    LocalSearch::Node *nodeToInsert = nullptr;

    for (auto *nodeU = routeU->depot->next; !nodeU->isDepot;
         nodeU = nodeU->next)
    {
        routeU = nodeU->route;
        auto *nodeX = nodeU->next;

        auto const routeUTwData = LocalSearch::TimeWindowData::merge(
            nodeU->prev->prefixTwData, nodeX->postfixTwData);
        int const costSuppU
            = params.dist(nodeU->prev->cour, nodeX->cour)
              - params.dist(nodeU->prev->cour, nodeU->cour)
              - params.dist(nodeU->cour, nodeX->cour)
              + penalties.load(routeU->load
                               - params.clients[nodeU->cour].demand)
              + penalties.timeWarp(routeUTwData) - routeU->penalty;

        for (auto *V = routeV->depot->next; !V->isDepot; V = V->next)
        {
            auto const routeVTwData = LocalSearch::TimeWindowData::merge(
                V->prefixTwData, nodeU->twData, V->next->postfixTwData);

            int const costSuppV
                = params.dist(V->cour, nodeU->cour)
                  + params.dist(nodeU->cour, V->next->cour)
                  - params.dist(V->cour, V->next->cour)
                  + penalties.load(routeV->load
                                   + params.clients[nodeU->cour].demand)
                  + penalties.timeWarp(routeVTwData) - routeV->penalty;

            if (costSuppU + costSuppV < bestCost)
            {
                bestCost = costSuppU + costSuppV;
                insertionPoint = V;
                nodeToInsert = nodeU;
            }
        }
    }

    if (!insertionPoint)
        return false;

    routeU = nodeToInsert->route;
    operators::insertNode(nodeToInsert, insertionPoint);
    nbMoves++;  // Increment move counter before updating route data
    searchCompleted = false;
    operators::updateRouteData(routeU, nbMoves, penalties, params);
    operators::updateRouteData(
        insertionPoint->route, nbMoves, penalties, params);

    return true;
}