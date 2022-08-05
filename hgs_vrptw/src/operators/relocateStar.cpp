#include "operators.h"

#include "TimeWindowSegment.h"

bool relocateStar(Route *routeU,
                  Route *routeV,
                  Penalties const &penalties)
{
    auto const &params = *routeU->params;

    int bestCost = 0;
    Node *insertionPoint = nullptr;
    Node *nodeToInsert = nullptr;

    for (auto *nodeU = routeU->depot->next; !nodeU->isDepot;
         nodeU = nodeU->next)
    {
        routeU = nodeU->route;
        auto *nodeX = nodeU->next;

        auto const routeUTwData
            = TimeWindowSegment::merge(nodeU->prev->twBefore, nodeX->twAfter);
        int const costSuppU
            = params.dist(nodeU->prev->client, nodeX->client)
              - params.dist(nodeU->prev->client, nodeU->client)
              - params.dist(nodeU->client, nodeX->client)
              + penalties.load(routeU->load
                               - params.clients[nodeU->client].demand)
              + penalties.timeWarp(routeUTwData) - routeU->penalty;

        for (auto *V = routeV->depot->next; !V->isDepot; V = V->next)
        {
            auto const routeVTwData = TimeWindowSegment::merge(
                V->twBefore, nodeU->tw, V->next->twAfter);

            int const costSuppV
                = params.dist(V->client, nodeU->client)
                  + params.dist(nodeU->client, V->next->client)
                  - params.dist(V->client, V->next->client)
                  + penalties.load(routeV->load
                                   + params.clients[nodeU->client].demand)
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

    operators::insertNode(nodeToInsert, insertionPoint);

    return true;
}