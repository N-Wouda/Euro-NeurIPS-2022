#include "Exchange.h"

#include "Route.h"
#include "TimeWindowSegment.h"

namespace
{
using TWS = TimeWindowSegment;
}

template <size_t N, size_t M>
bool Exchange<N, M>::isDepotInSegments(Node *U, Node *V) const
{
    auto eval = [&](Node *node, size_t chainLength)
    {
        for (size_t count = 0; count != chainLength; ++count, node = n(node))
            if (node->isDepot())
                return true;

        return false;
    };

    return eval(U, N) || eval(V, M);
}

template <size_t N, size_t M>
bool Exchange<N, M>::overlaps(Node *U, Node *V) const
{
    // clang-format off
    return U->route == V->route
        && U->position <= V->position + M - 1
        && V->position <= U->position + N - 1;
    // clang-format on
}

template <size_t N, size_t M> bool Exchange<N, M>::test(Node *U, Node *V)
{
    if (isDepotInSegments(U, V))
        return false;

    if (overlaps(U, V))  // cannot swap same route segments that overlap
        return false;

    if (M == 0 && U->position == V->position + 1)  // would insert U in same
        return false;                              // place it's now in

    auto const &params = *U->params;

    Node *segEndU = U;

    for (size_t count = 1; count != N; ++count)
        segEndU = n(segEndU);

    Node *segEndV = V;
    for (size_t count = 0; count != M; ++count)
        segEndV = n(segEndV);

    int proposed = Route::distBetween(V, segEndV)
                   + Route::distBetween(U, segEndU)
                   + params.dist(p(U)->client, V->client)
                   + params.dist(segEndV->client, n(segEndU)->client)
                   + params.dist(p(V)->client, U->client)
                   + params.dist(segEndU->client, n(segEndV)->client);

    int current = Route::distBetween(p(U), n(segEndU))
                  + Route::distBetween(p(V), n(segEndV));

    int deltaCost = proposed - current;

    if (U->route != V->route)
    {
        if (M == 0 && U->route->isFeasible() && deltaCost >= 0)
            return false;

        auto uTWS = TWS::merge(
            p(U)->twBefore, Route::twBetween(V, segEndV), n(segEndU)->twAfter);

        deltaCost += d_penalties->timeWarp(uTWS);
        deltaCost -= d_penalties->timeWarp(U->route->tw);

        auto const uLoad = Route::loadBetween(U, segEndU);
        auto const vLoad = Route::loadBetween(V, segEndV);
        auto const loadDiff = M == 0 ? uLoad : uLoad - vLoad;

        deltaCost += d_penalties->load(U->route->load - loadDiff);
        deltaCost -= d_penalties->load(U->route->load);

        if (M == 0 && deltaCost >= 0)  // never good if delta of just U's route
            return false;              // is not enough even without V

        deltaCost += d_penalties->load(V->route->load + loadDiff);
        deltaCost -= d_penalties->load(V->route->load);

        auto vTWS = TWS::merge(
            p(V)->twBefore, Route::twBetween(U, segEndU), n(segEndV)->twAfter);

        deltaCost += d_penalties->timeWarp(vTWS);
        deltaCost -= d_penalties->timeWarp(V->route->tw);
    }
    else  // within same route
    {
        if (!U->route->hasTimeWarp() && deltaCost >= 0)
            return false;

        if (U->position < V->position)
        {
            auto const tws = TWS::merge(p(U)->twBefore,
                                        Route::twBetween(V, segEndV),
                                        Route::twBetween(n(segEndU), p(V)),
                                        Route::twBetween(U, segEndU),
                                        n(segEndV)->twAfter);

            deltaCost += d_penalties->timeWarp(tws);
        }
        else
        {
            auto const tws = TWS::merge(p(V)->twBefore,
                                        Route::twBetween(U, segEndU),
                                        Route::twBetween(n(segEndV), p(U)),
                                        Route::twBetween(V, segEndV),
                                        n(segEndU)->twAfter);

            deltaCost += d_penalties->timeWarp(tws);
        }

        deltaCost -= d_penalties->timeWarp(U->route->tw);
    }

    return deltaCost < 0;
}

template <size_t N, size_t M> void Exchange<N, M>::apply(Node *U, Node *V)
{
    auto *insertVAfter = p(U);
    auto *insertUAfter = V;

    for (size_t count = 0; count != M; ++count)
        insertUAfter = n(V);

    auto *nU = U;

    // Find last element in U to insert into V, and then walk backwards.
    // This is necessary to ensure we get the same ordering in the route of
    // V as we had in U.
    for (size_t count = 0; count != N; ++count)
        nU = n(nU);

    for (size_t count = N; count != 0; --count)
    {
        auto *prev = p(nU);
        nU->insertAfter(insertUAfter);
        nU = prev;
    }

    // We now have a route V -> ... -> n(V) [M times] -> U -> ... -> n(U) [N
    // times]. Let's move the M nodes from V's route over to U.
    auto *nV = V;

    // Find last element in V to insert into U, and then walk backwards.
    // This is necessary to ensure we get the same ordering in the route of
    // U as we had in V.
    for (size_t count = 0; count != M; ++count)
        nV = n(nV);

    for (size_t count = M; count != 0; --count)
    {
        auto *prev = p(nV);
        nV->insertAfter(insertVAfter);
        nV = prev;
    }
}

// Explicit instantiations of the few moves we *might* want to have
template class Exchange<1, 0>;
template class Exchange<2, 0>;
template class Exchange<3, 0>;
template class Exchange<1, 1>;
template class Exchange<2, 1>;
template class Exchange<3, 1>;
template class Exchange<2, 2>;
template class Exchange<3, 2>;
template class Exchange<3, 3>;
