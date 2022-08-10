#include "Exchange.h"

#include "Route.h"
#include "TimeWindowSegment.h"

#include <iostream>

namespace
{
using TWS = TimeWindowSegment;
}

template <size_t N, size_t M>
std::pair<Node *, Node *> Exchange<N, M>::getEnds(Node *U, Node *V) const
{
    Node *endU = U;
    for (size_t count = 1; count != N; ++count)
        endU = n(endU);

    if constexpr (M == 0)
        return std::make_pair(endU, nullptr);
    else
    {
        Node *endV = V;
        for (size_t count = 1; count != M; ++count)
            endV = n(endV);

        return std::make_pair(endU, endV);
    }
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
    if (U->route == V->route)
        return false;

    if (isDepotInSegments(U, V))
        return false;

    if (overlaps(U, V))  // cannot swap same route segments that overlap
        return false;

    if (M == 0 && U->position == V->position + 1)  // would insert U in same
        return false;                              // place it's now in

    auto const &params = *U->params;
    auto const &[segEndU, segEndV] = getEnds(U, V);

    int proposed = 0;
    int current = 0;

    if constexpr (M == 0)  // nothing from V is moved, so we go to V
    {                      // -> U segment -> n(V)
        proposed += params.dist(V->client, U->client)
                    + Route::distBetween(U, segEndU)
                    + params.dist(segEndU->client, n(V)->client)
                    + params.dist(p(U)->client, n(segEndU)->client);

        current += Route::distBetween(p(U), n(segEndU))
                   + params.dist(V->client, n(V)->client);
    }
    else  // we also move something from V, so we go to p(U) -> V segment ->
    {     // stuff after U segment
        proposed += params.dist(p(U)->client, V->client)
                    + Route::distBetween(V, segEndV)
                    + params.dist(segEndV->client, n(segEndU)->client);

        // And similarly for the U segment inserted into V's route
        proposed += params.dist(p(V)->client, U->client)
                    + Route::distBetween(U, segEndU)
                    + params.dist(segEndU->client, n(segEndV)->client);

        current += Route::distBetween(p(U), n(segEndU))
                   + Route::distBetween(p(V), n(segEndV));
    }

    int deltaCost = proposed - current;

    if (U->route != V->route)
    {
        if constexpr (M == 0)
            if (U->route->isFeasible() && deltaCost >= 0)
                return false;

        TWS uTWS;

        if constexpr (M == 0)
            uTWS = TWS::merge(p(U)->twBefore, n(segEndU)->twAfter);
        else
            uTWS = TWS::merge(p(U)->twBefore,
                              Route::twBetween(V, segEndV),
                              n(segEndU)->twAfter);

        deltaCost += d_penalties->timeWarp(uTWS);
        deltaCost -= d_penalties->timeWarp(U->route->tw);

        auto const uLoad = Route::loadBetween(U, segEndU);
        auto const vLoad = M == 0 ? 0 : Route::loadBetween(V, segEndV);

        deltaCost += d_penalties->load(U->route->load + uLoad - vLoad);
        deltaCost -= d_penalties->load(U->route->load);

        if (M == 0 && deltaCost >= 0)  // never good if delta of just U's route
            return false;              // is not enough even without V

        deltaCost += d_penalties->load(V->route->load + uLoad - vLoad);
        deltaCost -= d_penalties->load(V->route->load);

        TWS vTWS;

        if constexpr (M == 0)
            vTWS = TWS::merge(
                p(V)->twBefore, Route::twBetween(U, segEndU), n(V)->twAfter);
        else
            vTWS = TWS::merge(p(V)->twBefore,
                              Route::twBetween(U, segEndU),
                              n(segEndV)->twAfter);

        deltaCost += d_penalties->timeWarp(vTWS);
        deltaCost -= d_penalties->timeWarp(V->route->tw);
    }
    else  // within same route
    {
        if (!U->route->hasTimeWarp() && deltaCost >= 0)
            return false;

        if (U->position < V->position)
        {
            TWS tws;

            if constexpr (M == 0)
                tws = TWS::merge(p(U)->twBefore,
                                 Route::twBetween(n(segEndU), V),
                                 Route::twBetween(U, segEndU),
                                 n(V)->twAfter);
            else
            {
                if (U->position + N == V->position)
                    tws = TWS::merge(p(U)->twBefore,
                                     Route::twBetween(V, segEndV),
                                     Route::twBetween(U, segEndU),
                                     n(segEndV)->twAfter);
                else
                    tws = TWS::merge(p(U)->twBefore,
                                     Route::twBetween(V, segEndV),
                                     Route::twBetween(n(segEndU), p(V)),
                                     Route::twBetween(U, segEndU),
                                     n(segEndV)->twAfter);
            }

            deltaCost += d_penalties->timeWarp(tws);
        }
        else
        {
            TWS tws;

            if constexpr (M == 0)
                tws = TWS::merge(V->twBefore,
                                 Route::twBetween(U, segEndU),
                                 Route::twBetween(n(V), p(U)),
                                 n(segEndU)->twAfter);
            else
            {
                if (V->position + M == U->position)
                    tws = TWS::merge(p(V)->twBefore,
                                     Route::twBetween(U, segEndU),
                                     Route::twBetween(V, segEndV),
                                     n(segEndU)->twAfter);
                else
                    tws = TWS::merge(p(V)->twBefore,
                                     Route::twBetween(U, segEndU),
                                     Route::twBetween(n(segEndV), p(U)),
                                     Route::twBetween(V, segEndV),
                                     n(segEndU)->twAfter);
            }

            deltaCost += d_penalties->timeWarp(tws);
        }

        deltaCost -= d_penalties->timeWarp(U->route->tw);
    }

    return deltaCost < 0;
}

template <size_t N, size_t M> void Exchange<N, M>::apply(Node *U, Node *V)
{
    std::cout << U->route->idx << ' ' << V->route->idx << '\n';
    std::cout << U->client << ' ' << V->client << '\n';

    for (Node *v = V->route->depot->next; !v->isDepot(); v = v->next)
        std::cout << v->client << ' ';
    std::cout << '\n';

    for (Node *u = U->route->depot->next; !u->isDepot(); u = u->next)
        std::cout << u->client << ' ';
    std::cout << '\n';

    auto const &[segEndU, segEndV] = getEnds(U, V);

    for (size_t count = 0; count != N - M; ++count)
        segEndU->insertAfter(segEndV ? segEndV : V);

    Node *nU = U;
    Node *nV = V;
    for (size_t count = 0; count != std::min(N, M); ++count)
    {
        nU->swapWith(nV);  // This loop flip-flops between storing V in U, and
        nU = nU->next;     // vice versa; that's OK.
        nV = nV->next;
    }

    for (Node *u = U->route->depot->next; !u->isDepot(); u = u->next)
        std::cout << u->client << ' ';
    std::cout << '\n';

    for (Node *v = V->route->depot->next; !v->isDepot(); v = v->next)
        std::cout << v->client << ' ';
    std::cout << '\n';
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
