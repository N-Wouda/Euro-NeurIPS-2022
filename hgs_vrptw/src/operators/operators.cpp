#include "operators.h"

#include "Node.h"
#include "Route.h"
#include "TimeWindowSegment.h"

#include <cmath>

void operators::insertNode(Node *U, Node *V)
{
    U->prev->next = U->next;
    U->next->prev = U->prev;
    V->next->prev = U;
    U->prev = V;
    U->next = V->next;
    V->next = U;
    U->route = V->route;
}

void operators::swapNode(Node *U, Node *V)
{
    auto *myVPred = V->prev;
    auto *myVSuiv = V->next;
    auto *myUPred = U->prev;
    auto *myUSuiv = U->next;
    auto *myRouteU = U->route;
    auto *myRouteV = V->route;

    myUPred->next = V;
    myUSuiv->prev = V;
    myVPred->next = U;
    myVSuiv->prev = U;

    U->prev = myVPred;
    U->next = myVSuiv;
    V->prev = myUPred;
    V->next = myUSuiv;

    U->route = myRouteV;
    V->route = myRouteU;
}
