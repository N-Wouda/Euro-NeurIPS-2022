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
    auto *VPred = V->prev;
    auto *VSucc = V->next;
    auto *UPred = U->prev;
    auto *USucc = U->next;

    auto *routeU = U->route;
    auto *routeV = V->route;

    UPred->next = V;
    USucc->prev = V;
    VPred->next = U;
    VSucc->prev = U;

    U->prev = VPred;
    U->next = VSucc;
    V->prev = UPred;
    V->next = USucc;

    U->route = routeV;
    V->route = routeU;
}
