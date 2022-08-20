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
    auto const oldNodes = nodes;

    load = 0;
    nodes.clear();

    int distance = 0;
    int reverseDistance = 0;

    setupNodes();
    setupSector();
    setupRouteTimeWindows();

    bool foundChange = false;

    for (size_t pos = 0; pos != nodes.size(); ++pos)
    {
        auto *node = nodes[pos];

        if (!foundChange && (pos >= oldNodes.size() || node != oldNodes[pos]))
        {
            foundChange = true;

            if (pos > 0)  // change at pos, so everything before is the same
            {
                load = nodes[pos - 1]->cumulatedLoad;
                distance = nodes[pos - 1]->cumulatedDistance;
                reverseDistance = nodes[pos - 1]->cumulatedReversalDistance;
            }

            for (size_t jumpIdx = 0; jumpIdx != jumps.size(); ++jumpIdx)
            {
                if (pos <= jumpPts[jumpIdx])
                {
                    jumps[jumpIdx].clear();
                    continue;
                }

                auto const jumpSize = jumpPts[jumpIdx];
                size_t offset = pos - jumpSize;

                auto &jumpList = jumps[jumpIdx];
                jumpList.erase(jumpList.begin() + offset, jumpList.end());
            }
        }

        if (!foundChange)
            continue;

        load += params->clients[node->client].demand;
        distance += params->dist(p(node)->client, node->client);

        reverseDistance += params->dist(node->client, p(node)->client);
        reverseDistance -= params->dist(p(node)->client, node->client);

        node->position = pos + 1;
        node->cumulatedLoad = load;
        node->cumulatedDistance = distance;
        node->cumulatedReversalDistance = reverseDistance;

        installJumpPoints(node);
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

void Route::setupRouteTimeWindows()
{
    for (auto *node : nodes)  // backward time window data
        node->twBefore = TWS::merge(p(node)->twBefore, node->tw);

    auto *node = nodes.back();
    tw = node->twBefore;  // whole route time window data

    do  // forward time window data
    {
        auto *prev = p(node);
        prev->twAfter = TWS::merge(prev->tw, node->twAfter);
        node = prev;
    } while (!node->isDepot());
}

void Route::setupSector()
{
    if (empty())
    {
        angleCenter = 1.e30;
        return;
    }

    sector.initialize(params->clients[n(depot)->client].angle);

    int cumulatedX = 0;
    int cumulatedY = 0;

    for (auto *node : nodes)
    {
        if (!node->isDepot())
        {
            cumulatedX += params->clients[node->client].x;
            cumulatedY += params->clients[node->client].y;

            sector.extend(params->clients[node->client].angle);
        }
    }

    angleCenter = atan2(
        cumulatedY / static_cast<double>(size()) - params->clients[0].y,
        cumulatedX / static_cast<double>(size()) - params->clients[0].x);

    // Enforce minimum size of circle sector
    if (params->config.minCircleSectorSize > 0)
    {
        int const growSectorBy = (params->config.minCircleSectorSize
                                  - CircleSector::positive_mod(sector) + 1)
                                 / 2;

        if (growSectorBy > 0)
        {
            sector.extend(sector.start - growSectorBy);
            sector.extend(sector.end + growSectorBy);
        }
    }
}

std::ostream &operator<<(std::ostream &out, Route const &route)
{
    out << "Route #" << route.idx + 1 << ":";  // route number
    for (auto *node = n(route.depot); !node->isDepot(); node = n(node))
        out << ' ' << node->client;  // client index
    out << '\n';

    return out;
}
