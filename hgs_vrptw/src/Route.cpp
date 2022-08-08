#include "Route.h"
#include "fatan.h"

#include <bit>
#include <cassert>

namespace
{
using TWS = TimeWindowSegment;
}

void Route::update()
{
    size_t const prevSize = nodes.size();

    load = 0;
    jumps = {{}, {}};
    jumps[0].reserve(prevSize);  // Route's size has likely changed since the
    jumps[1].reserve(prevSize);  // last update, but probably not *much*.

    nodes = {};
    nodes.reserve(prevSize);

    size_t place = 0;
    int reverseDistance = 0;
    int cumulatedX = 0;
    int cumulatedY = 0;

    auto *node = depot;
    node->position = 0;
    node->cumulatedLoad = 0;
    node->cumulatedReversalDistance = 0;

    if (!node->next->isDepot())
        sector.initialize(params->clients[node->next->client].angle);

    do
    {
        node = node->next;
        nodes.push_back(node);

        place++;
        node->position = place;

        load += params->clients[node->client].demand;
        node->cumulatedLoad = load;

        reverseDistance += params->dist(node->client, node->prev->client);
        reverseDistance -= params->dist(node->prev->client, node->client);

        node->cumulatedReversalDistance = reverseDistance;

        node->twBefore = TWS::merge(node->prev->twBefore, node->tw);

        if (!node->isDepot())
        {
            cumulatedX += params->clients[node->client].x;
            cumulatedY += params->clients[node->client].y;

            sector.extend(params->clients[node->client].angle);
        }

        installJumpPoints(node);
    } while (!node->isDepot());

    tw = node->twBefore;
    nbCustomers = place - 1;

    // Time window data in reverse direction, node should be end depot now
    do
    {
        node = node->prev;
        node->twAfter = TWS::merge(node->tw, node->next->twAfter);
    } while (!node->isDepot());

    if (empty())
    {
        angleCenter = 1.e30;
        return;
    }

    angleCenter = fatan2(
        cumulatedY / static_cast<double>(nbCustomers) - params->clients[0].y,
        cumulatedX / static_cast<double>(nbCustomers) - params->clients[0].x);

    // Enforce minimum size of circle sector
    if (params->config.minCircleSectorSize > 0)
    {
        const int growSectorBy = (params->config.minCircleSectorSize
                                  - CircleSector::positive_mod(sector) + 1)
                                 / 2;

        if (growSectorBy > 0)
        {
            sector.extend(sector.start - growSectorBy);
            sector.extend(sector.end + growSectorBy);
        }
    }
}

TimeWindowSegment Route::twBetween(Node const *start, Node const *end)
{
    assert(start->route == end->route);
    assert(start->position <= end->position);

    if (start->isDepot())
        return end->twBefore;

    if (end->isDepot())
        return start->twAfter;

    Node const *node = start;
    TimeWindowSegment data = start->tw;
    auto const &jumps = start->route->jumps;

    while (node != end)
    {
        auto const dist = end->position - node->position;

        if (dist >= jumpPts.front())  // can make at least one jump
        {
            auto const pos = std::bit_floor(std::min(dist, jumpPts.back()));
            auto const &jumpList = jumps[std::bit_width(pos) - jumpOffset];

            if (jumpList.size() >= node->position)  // jump list contains node
            {
                auto const &jumpNode = jumpList[node->position - 1];
                data = TWS::merge(data, jumpNode.tw);
                node = jumpNode.to;

                continue;
            }
        }

        node = node->next;
        data = TWS::merge(data, node->tw);
    }

    return data;
}

void Route::installJumpPoints(Node const *node)
{
    JumpNode const *toNextJump = nullptr;

    for (size_t const position : jumpPts)
        if (node->position > position && nodes.size() > position)
        {
            auto *prev = nodes[node->position - position - 1];
            auto const idx_ = std::bit_width(position) - jumpOffset;

            if (toNextJump)
            {
                // After installing earlier, part way jumps, part of the way
                // of larger jumps is already known. We reuse that by computing
                // the time window between (prev, part way) separately and then
                // merging that with (part way, node) which we already know.
                auto const prev2jump = Route::twBetween(prev, toNextJump->from);
                auto const prev2node = TWS::merge(prev2jump, toNextJump->tw);
                toNextJump = &jumps[idx_].emplace_back(prev, node, prev2node);
            }
            else
                toNextJump = &jumps[idx_].emplace_back(prev, node);
        }
}
