#include "Route.h"

#include <cmath>
#include <ostream>

namespace
{
using TWS = TimeWindowSegment;
}

void Route::update()
{
    auto const oldNodes = nodes;
    setupNodes();

    int load = 0;
    int distance = 0;
    int reverseDistance = 0;
    bool foundChange = false;

    for (size_t pos = 0; pos != nodes.size(); ++pos)
    {
        auto *node = nodes[pos];

        if (!foundChange && (pos >= oldNodes.size() || node != oldNodes[pos]))
        {
            foundChange = true;

            if (pos > 0)  // change at pos, so everything before is the same
            {             // and we can re-use cumulative calculations
                load = nodes[pos - 1]->cumulatedLoad;
                distance = nodes[pos - 1]->cumulatedDistance;
                reverseDistance = nodes[pos - 1]->cumulatedReversalDistance;
            }

            if (pos <= jumpDistance)
                jumps.clear();
            else
                jumps.erase(jumps.begin() + pos - jumpDistance, jumps.end());
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
        node->twBefore = TWS::merge(p(node)->twBefore, node->tw);

        if (node->position > jumpDistance && nodes.size() > jumpDistance)
        {
            // We cannot yet use Route::twBetween here since the jumps are
            // obviously not yet available.
            auto *prev = nodes[pos - jumpDistance];
            auto tws = prev->tw;

            for (auto step = prev->position; step != node->position; ++step)
                tws = TWS::merge(tws, nodes[step]->tw);

            jumps.emplace_back(tws);
        }
    }

    setupSector();
    setupRouteTimeWindows();
}

TimeWindowSegment Route::twBetween(Node const *start, Node const *end)
{
    assert(start->route == end->route);
    assert(start->position <= end->position);

    if (start->isDepot())
        return end->twBefore;

    if (end->isDepot())
        return start->twAfter;

    auto step = start->position;
    auto data = start->tw;

    auto const *route = start->route;
    auto const &jumps = route->jumps;

    // Jump as much as we can...
    for (; step + jumpDistance <= end->position; step += jumpDistance)
        data = TWS::merge(data, jumps[step - 1]);

    // ...and do the rest in one-step updates.
    for (; step != end->position; ++step)
        data = TWS::merge(data, route->nodes[step]->tw);

    return data;
}

void Route::setupNodes()
{
    nodes.clear();
    auto *node = depot;

    do
    {
        node = n(node);
        nodes.push_back(node);
    } while (!node->isDepot());
}

void Route::setupRouteTimeWindows()
{
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

    // This computes a pseudo-angle that sorts roughly equivalently to the atan2
    // angle, but is much faster to compute. See the following post for details:
    // https://stackoverflow.com/a/16561333/4316405.
    auto dy = cumulatedY / static_cast<double>(size()) - params->clients[0].y;
    auto dx = cumulatedX / static_cast<double>(size()) - params->clients[0].x;
    angleCenter = std::copysign(1. - dx / (std::fabs(dx) + std::fabs(dy)), dy);

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
