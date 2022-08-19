#include "Route.h"

#include <bit>
#include <cmath>
#include <ostream>

namespace
{
using TWS = TimeWindowSegment;
}

void Route::update()
{
    // TODO simplify updating
    auto const oldNodes = nodes;

    load = 0;
    nodes.clear();

    int distance = 0;
    int reverseDistance = 0;
    int cumulatedX = 0;
    int cumulatedY = 0;

    depot->position = 0;
    depot->cumulatedLoad = 0;
    depot->cumulatedDistance = 0;
    depot->cumulatedReversalDistance = 0;

    if (!n(depot)->isDepot())
        sector.initialize(params->clients[n(depot)->client].angle);
    else
        sector.initialize(params->clients[0].angle);

    setupNodes();

    jumps[0].clear();
    jumps[1].clear();

//    size_t firstChangedPos = 1;
    bool foundChange = false;

    for (size_t idx_ = 0; idx_ != nodes.size(); ++idx_)
    {
        auto *node = nodes[idx_];

        if (!foundChange && (idx_ >= oldNodes.size() || node != oldNodes[idx_]))
        {
            foundChange = true;
//            firstChangedPos = idx_ + 1;
        }

        // TODO use unchanged for caching

        if (!node->isDepot())
        {
            cumulatedX += params->clients[node->client].x;
            cumulatedY += params->clients[node->client].y;

            sector.extend(params->clients[node->client].angle);
        }

        load += params->clients[node->client].demand;
        distance += params->dist(p(node)->client, node->client);

        reverseDistance += params->dist(node->client, p(node)->client);
        reverseDistance -= params->dist(p(node)->client, node->client);

        node->position = idx_ + 1;
        node->cumulatedLoad = load;
        node->cumulatedDistance = distance;
        node->cumulatedReversalDistance = reverseDistance;
        node->twBefore = TWS::merge(p(node)->twBefore, node->tw);

        installJumpPoints(node);
    }

    auto *node = nodes.back();
    tw = node->twBefore;

    // Time window data in reverse direction, node should be end depot now
    do
    {
        node = p(node);
        node->twAfter = TWS::merge(node->tw, n(node)->twAfter);
    } while (!node->isDepot());

    if (empty())
    {
        angleCenter = 1.e30;
        return;
    }

    angleCenter = atan2(
        cumulatedY / static_cast<double>(size()) - params->clients[0].y,
        cumulatedX / static_cast<double>(size()) - params->clients[0].x);

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

void Route::setupNodes()
{
    auto *node = depot;

    do
    {
        node = n(node);
        nodes.push_back(node);
    } while (!node->isDepot());
}

std::ostream &operator<<(std::ostream &out, Route const &route)
{
    out << "Route #" << route.idx + 1 << ":";  // route number
    for (auto *node = n(route.depot); !node->isDepot(); node = n(node))
        out << ' ' << node->client;  // client index
    out << '\n';

    return out;
}
