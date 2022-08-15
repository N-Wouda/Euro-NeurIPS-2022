#include "ReverseExchange.h"

#include "Route.h"
#include "TimeWindowSegment.h"

namespace
{
using TWS = TimeWindowSegment;
}

template <size_t N, size_t M>
std::pair<Node *, Node *> ReverseExchange<N, M>::getEnds(Node *U, Node *V) const
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
bool ReverseExchange<N, M>::isDepotInSegments(Node *U, Node *V) const
{
    auto eval = [&](Node *node, size_t chainLength) {
        for (size_t count = 0; count != chainLength; ++count, node = n(node))
            if (node->isDepot())
                return true;

        return false;
    };

    return eval(U, N) || eval(V, M);
}

template <size_t N, size_t M>
bool ReverseExchange<N, M>::overlaps(Node *U, Node *V) const
{
    // clang-format off
    return U->route == V->route
        && U->position <= V->position + M - 1
        && V->position <= U->position + N - 1;
    // clang-format on
}

template <size_t N, size_t M>
bool ReverseExchange<N, M>::adjacent(Node *U, Node *V) const
{
    if (U->route != V->route)
        return false;

    return U->position + N == V->position || V->position + M == U->position;
}

template <size_t N, size_t M>
bool ReverseExchange<N, M>::testPureMove(Node *U, Node *V) const
{
    if (isDepotInSegments(U, V) || overlaps(U, V) || U == n(V))
        return false;

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

        auto const loadDiff = Route::loadBetween(U, endU);

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
    else  // within same route
    {
        if (!U->route->hasTimeWarp() && deltaCost >= 0)
            return false;

        if (U->position < V->position)
        {
            auto const tws = TWS::merge(p(U)->twBefore,
                                        Route::twBetween(n(endU), V),
                                        Route::twBetween(U, endU),
                                        n(V)->twAfter);

            deltaCost += d_penalties->timeWarp(tws);
        }
        else
        {
            auto const tws = TWS::merge(V->twBefore,
                                        Route::twBetween(U, endU),
                                        Route::twBetween(n(V), p(U)),
                                        n(endU)->twAfter);

            deltaCost += d_penalties->timeWarp(tws);
        }

        deltaCost -= d_penalties->timeWarp(U->route->tw);
    }

    return deltaCost < 0;
}

template <size_t N, size_t M>
bool ReverseExchange<N, M>::testSwapMove(Node *U, Node *V) const
{
    if constexpr (N == M)  // symmetric, so only have to evaluate this once
        if (U->client >= V->client)
            return false;

    if (isDepotInSegments(U, V) || overlaps(U, V) || adjacent(U, V))
        return false;

    auto const &params = *U->params;
    auto const [endU, endV] = getEnds(U, V);

    int const current
        = Route::distBetween(p(U), n(endU)) + Route::distBetween(p(V), n(endV));

    int const proposed
        //   p(U) -> V -> ... -> endV -> n(endU)
        // + p(V) -> U -> ... -> endU -> n(endV)
        = params.dist(p(U)->client, V->client) + Route::distBetween(V, endV)
          + params.dist(endV->client, n(endU)->client)
          + params.dist(p(V)->client, U->client) + Route::distBetween(U, endU)
          + params.dist(endU->client, n(endV)->client);

    int deltaCost = proposed - current;

    if (U->route != V->route)
    {
        if (U->route->isFeasible() && V->route->isFeasible() && deltaCost >= 0)
            return false;

        auto uTWS = TWS::merge(
            p(U)->twBefore, Route::twBetween(V, endV), n(endU)->twAfter);

        deltaCost += d_penalties->timeWarp(uTWS);
        deltaCost -= d_penalties->timeWarp(U->route->tw);

        auto vTWS = TWS::merge(
            p(V)->twBefore, Route::twBetween(U, endU), n(endV)->twAfter);

        deltaCost += d_penalties->timeWarp(vTWS);
        deltaCost -= d_penalties->timeWarp(V->route->tw);

        auto const loadU = Route::loadBetween(U, endU);
        auto const loadV = Route::loadBetween(V, endV);
        auto const loadDiff = loadU - loadV;

        deltaCost += d_penalties->load(U->route->load - loadDiff);
        deltaCost -= d_penalties->load(U->route->load);

        deltaCost += d_penalties->load(V->route->load + loadDiff);
        deltaCost -= d_penalties->load(V->route->load);
    }
    else  // within same route
    {
        if (!U->route->hasTimeWarp() && deltaCost >= 0)
            return false;

        if (U->position < V->position)
        {
            auto const tws = TWS::merge(p(U)->twBefore,
                                        Route::twBetween(V, endV),
                                        Route::twBetween(n(endU), p(V)),
                                        Route::twBetween(U, endU),
                                        n(endV)->twAfter);

            deltaCost += d_penalties->timeWarp(tws);
        }
        else
        {
            auto const tws = TWS::merge(p(V)->twBefore,
                                        Route::twBetween(U, endU),
                                        Route::twBetween(n(endV), p(U)),
                                        Route::twBetween(V, endV),
                                        n(endU)->twAfter);

            deltaCost += d_penalties->timeWarp(tws);
        }

        deltaCost -= d_penalties->timeWarp(U->route->tw);
    }

    return deltaCost < 0;
}

template <size_t N, size_t M> bool ReverseExchange<N, M>::test(Node *U, Node *V)
{
    if constexpr (M == 0)  // special case where nothing in V is moved
        return testPureMove(U, V);
    else
        return testSwapMove(U, V);
}

template <size_t N, size_t M>
void ReverseExchange<N, M>::apply(Node *U, Node *V)
{
    auto const [endU, endV] = getEnds(U, V);

    auto *insertUAfter = M == 0 ? V : endV;
    auto *uToInsert = endU;

    // Insert these 'extra' nodes of U after the end of V...
    for (size_t count = 0; count != N - M; ++count)
    {
        auto *prev = uToInsert->prev;
        uToInsert->insertAfter(insertUAfter);
        uToInsert = prev;
    }

    // ...and swap the overlapping nodes!
    for (size_t count = 0; count != std::min(N, M); ++count)
    {
        auto *nextU = U->next;
        auto *nextV = V->next;
        U->swapWith(V);
        U = nextU;
        V = nextV;
    }
}

// Explicit instantiations of the few moves we *might* want to have
template class ReverseExchange<1, 0>;
template class ReverseExchange<2, 0>;
template class ReverseExchange<3, 0>;
template class ReverseExchange<1, 1>;
template class ReverseExchange<2, 1>;
template class ReverseExchange<3, 1>;
template class ReverseExchange<2, 2>;
template class ReverseExchange<3, 2>;
template class ReverseExchange<3, 3>;
