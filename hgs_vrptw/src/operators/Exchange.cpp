#include "Exchange.h"

#include "Route.h"
#include "TimeWindowSegment.h"

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

    Node *endV = V;
    for (size_t count = 1; count != M; ++count)
        endV = n(endV);

    return std::make_pair(endU, endV);
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

template <size_t N, size_t M>
bool Exchange<N, M>::testPureMove(Node *U, Node *V) const
{
    auto const [endU, _] = getEnds(U, V);
    auto const &params = *U->params;

    int const current = Route::distBetween(p(U), n(endU))
                        + params.dist(V->client, n(V)->client);

    int const proposed = params.dist(V->client, U->client)
                         + Route::distBetween(U, endU)
                         + params.dist(endU->client, n(V)->client)
                         + params.dist(p(U)->client, n(endU)->client);

    int deltaCost = proposed - current;

    if (U->route != V->route)
    {
        if (U->route->isFeasible() && deltaCost >= 0)
            return false;

        auto uTWS = TWS::merge(p(U)->twBefore, n(endU)->twAfter);

        deltaCost += d_penalties->timeWarp(uTWS);
        deltaCost -= d_penalties->timeWarp(U->route->tw);

        auto const loadDiff = Route::loadBetween(p(U), endU);

        deltaCost += d_penalties->load(U->route->load - loadDiff);
        deltaCost -= d_penalties->load(U->route->load);

        if (deltaCost >= 0)  // if delta cost of just U's route is not enough
            return false;    // even without V, the move will never be good

        deltaCost += d_penalties->load(V->route->load + loadDiff);
        deltaCost -= d_penalties->load(V->route->load);

        auto vTWS
            = TWS::merge(V->twBefore, Route::twBetween(U, endU), n(V)->twAfter);

        deltaCost += d_penalties->timeWarp(vTWS);
        deltaCost -= d_penalties->timeWarp(V->route->tw);
    }
    else
    {
        if (!U->route->hasTimeWarp() && deltaCost >= 0)
            return false;

        if (U->position < V->position)
        {
            auto const uTWS = TWS::merge(p(U)->twBefore,
                                         Route::twBetween(n(endU), V),
                                         Route::twBetween(U, endU),
                                         n(V)->twAfter);

            deltaCost += d_penalties->timeWarp(uTWS);
        }
        else
        {
            auto const uTWS = TWS::merge(V->twBefore,
                                         Route::twBetween(U, endU),
                                         Route::twBetween(n(V), p(U)),
                                         n(endU)->twAfter);

            deltaCost += d_penalties->timeWarp(uTWS);
        }

        deltaCost -= d_penalties->timeWarp(U->route->tw);
    }

    return deltaCost < 0;
}

template <size_t N, size_t M> bool Exchange<N, M>::test(Node *U, Node *V)
{
    if (isDepotInSegments(U, V) || overlaps(U, V))
        return false;

    if constexpr (M == 0)
        return testPureMove(U, V);

    // TODO
    return false;
}

template <size_t N, size_t M> void Exchange<N, M>::apply(Node *U, Node *V)
{
    auto const [endU, endV] = getEnds(U, V);
    auto *nU = endU;

    for (auto count = 0; count != N; ++count)
    {
        nU->insertAfter(V);
        nU = nU->prev;
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
