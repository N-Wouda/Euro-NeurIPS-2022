#include "operators.h"

bool swapStar(Route *routeU, Route *routeV, Penalties const &penalties)
{
    // Preprocessing phase
    for (auto *U = routeU->depot->next; !U->isDepot; U = U->next)
        for (auto *V = routeV->depot->next; !V->isDepot; V = V->next)
            ;

    for (auto *V = routeV->depot->next; !V->isDepot; V = V->next)
        for (auto *U = routeU->depot->next; !U->isDepot; U = U->next)
            ;

    // Search phase
    for (auto *U = routeU->depot->next; !U->isDepot; U = U->next)
        for (auto *V = routeV->depot->next; !V->isDepot; V = V->next)
            ;

    return false;
}
