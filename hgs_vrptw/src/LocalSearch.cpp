#include "LocalSearch.h"

#include "Individual.h"
#include "Params.h"

#include <numeric>
#include <stdexcept>
#include <vector>

void LocalSearch::operator()(Individual &indiv)
{
    // Shuffling the node order beforehand adds diversity to the search
    std::shuffle(orderNodes.begin(), orderNodes.end(), rng);
    std::shuffle(orderRoutes.begin(), orderRoutes.end(), rng);

    loadIndividual(indiv);       // load individual...
    search();                    // ...perform local search...
    indiv = exportIndividual();  // ...export result back into the individual
}

void LocalSearch::search()
{
    if (nodeOps.empty() && routeOps.empty())
        throw std::runtime_error("No known node or route operators.");

    // Caches the last time node or routes were tested for modification (uses
    // nbMoves to track this). The lastModified field, in contrast, track when
    // a route was last *actually* modified.
    std::vector<int> lastTestedNodes(params.nbClients + 1, -1);
    std::vector<int> lastTestedRoutes(params.nbVehicles, -1);
    lastModified = std::vector<int>(params.nbVehicles, 0);
    nbMoves = 0;

    // At least two iterations as empty routes are not evaluated in the first
    for (int step = 0; step <= 1 || !searchCompleted; ++step)
    {
        searchCompleted = true;

        // Node operators are evaluated at neighbouring (U, V) pairs.
        for (int const uClient : orderNodes)
        {
            Node *U = &clients[uClient];
            auto const lastTestedNode = lastTestedNodes[U->client];
            lastTestedNodes[U->client] = nbMoves;

            // Shuffling the neighbours in this loop should not matter much as
            // we are already randomizing the nodes U.
            for (auto const vClient : params.getNeighboursOf(U->client))
            {
                Node *V = &clients[vClient];

                auto const lastModifiedRoute = std::max(
                    lastModified[U->route->idx], lastModified[V->route->idx]);

                // Evaluate operators only if routes have changed recently.
                if (step == 0 || lastModifiedRoute > lastTestedNode)
                {
                    if (applyNodeOperators(U, V))
                        continue;

                    if (p(V)->isDepot() && applyNodeOperators(U, p(V)))
                        continue;
                }
            }

            // Empty route moves are not tested in the first iteration to avoid
            // increasing the fleet size too much.
            if (step > 0)
            {
                auto pred = [](auto const &route) { return route.empty(); };
                auto empty = std::find_if(routes.begin(), routes.end(), pred);

                if (empty == routes.end())
                    continue;

                if (applyNodeOperators(U, empty->depot))
                    continue;
            }
        }

        // Route operators are evaluated only after node operators get stuck,
        // and only sometimes when we want to intensify the search.
        if (searchCompleted)
            for (int const rU : orderRoutes)
            {
                auto &U = routes[rU];

                if (U.empty())
                    continue;

                auto const lastTested = lastTestedRoutes[U.idx];
                lastTestedRoutes[U.idx] = nbMoves;

                // Shuffling in this loop should not matter much as we are
                // already randomizing the routes U.
                for (int rV = 0; rV != U.idx; ++rV)
                {
                    auto &V = routes[rV];

                    if (V.empty() || !U.overlapsWith(V))
                        continue;

                    auto const lastModifiedRoute
                        = std::max(lastModified[U.idx], lastModified[V.idx]);

                    if (step > 0 && lastModifiedRoute <= lastTested)
                        continue;

                    if (applyRouteOperators(&U, &V))
                        continue;
                }
            }
    }
}

bool LocalSearch::applyNodeOperators(Node *U, Node *V)
{
    for (auto op : nodeOps)
        if (op->evaluate(U, V) < 0)
        {
            auto *routeU = U->route;  // copy pointers because the operator can
            auto *routeV = V->route;  // modify the node's route membership

            op->apply(U, V);
            update(routeU, routeV);

            return true;
        }

    return false;
}

bool LocalSearch::applyRouteOperators(Route *U, Route *V)
{
    auto score = -0.24;
    score += 0.14 * U->hasTimeWarp();
    score += 0.12 * V->hasTimeWarp();
    score += 0.19 * U->hasExcessCapacity();
    score += 0.12 * V->hasExcessCapacity();
    score += -0.01 * U->size() / static_cast<double>(params.nbClients + 1);
    score += -0.03 * V->size() / static_cast<double>(params.nbClients + 1);
    score += 0.24 * (V->angleCenter - U->angleCenter);

    auto dist = INT_MAX;

    for (size_t uIdx = 0; uIdx != U->size(); ++uIdx)
        for (size_t vIdx = 0; vIdx != V->size(); ++vIdx)
        {
            auto *nodeU = (*U)[uIdx + 1];
            auto *nodeV = (*V)[vIdx + 1];

            if (params.dist(nodeU->client, nodeV->client) < dist)
                dist = params.dist(nodeU->client, nodeV->client);

            if (params.dist(nodeV->client, nodeU->client) < dist)
                dist = params.dist(nodeV->client, nodeU->client);
        }

    score += -0.85 * dist / static_cast<double>(params.maxDist());

    if (score < 0)
        return false;

    for (auto op : routeOps)
        if (op->evaluate(U, V) < 0)
        {
            op->apply(U, V);
            update(U, V);

            return true;
        }

    return false;
}

void LocalSearch::update(Route *U, Route *V)
{
    nbMoves++;
    searchCompleted = false;

    U->update();
    lastModified[U->idx] = nbMoves;

    for (auto op : routeOps)  // TODO only route operators use this (SWAP*).
        op->update(U);        //  Maybe later also expand to node ops?

    if (U != V)
    {
        V->update();
        lastModified[V->idx] = nbMoves;

        for (auto op : routeOps)
            op->update(V);
    }
}

void LocalSearch::loadIndividual(Individual const &indiv)
{
    for (int client = 0; client <= params.nbClients; client++)
        clients[client].tw = {&params,
                              client,
                              client,
                              params.clients[client].servDur,
                              0,
                              params.clients[client].twEarly,
                              params.clients[client].twLate,
                              params.clients[client].releaseTime};

    auto const &routesIndiv = indiv.getRoutes();

    for (int r = 0; r < params.nbVehicles; r++)
    {
        Node *startDepot = &startDepots[r];
        Node *endDepot = &endDepots[r];

        startDepot->prev = endDepot;
        startDepot->next = endDepot;

        endDepot->prev = startDepot;
        endDepot->next = startDepot;

        startDepot->tw = clients[0].tw;
        startDepot->twBefore = clients[0].tw;
        startDepot->twAfter = clients[0].tw;

        endDepot->tw = clients[0].tw;
        endDepot->twBefore = clients[0].tw;
        endDepot->twAfter = clients[0].tw;

        Route *route = &routes[r];

        if (!routesIndiv[r].empty())
        {
            Node *client = &clients[routesIndiv[r][0]];
            client->route = route;

            client->prev = startDepot;
            startDepot->next = client;

            for (int i = 1; i < static_cast<int>(routesIndiv[r].size()); i++)
            {
                Node *prev = client;

                client = &clients[routesIndiv[r][i]];
                client->route = route;

                client->prev = prev;
                prev->next = client;
            }

            client->next = endDepot;
            endDepot->prev = client;
        }

        route->update();
    }

    for (auto op : nodeOps)
        op->init(indiv);

    for (auto op : routeOps)
        op->init(indiv);
}

Individual LocalSearch::exportIndividual()
{
    std::vector<std::pair<double, int>> routePolarAngles;
    routePolarAngles.reserve(params.nbVehicles);

    for (int r = 0; r < params.nbVehicles; r++)
        routePolarAngles.emplace_back(routes[r].angleCenter, r);

    // Empty routes have a large center angle, and thus always sort at the end
    std::sort(routePolarAngles.begin(), routePolarAngles.end());

    std::vector<std::vector<int>> indivRoutes(params.nbVehicles);

    for (int r = 0; r < params.nbVehicles; r++)
    {
        Node *node = startDepots[routePolarAngles[r].second].next;

        while (!node->isDepot())
        {
            indivRoutes[r].push_back(node->client);
            node = node->next;
        }
    }

    return {&params, indivRoutes};
}

LocalSearch::LocalSearch(Params &params, XorShift128 &rng)
    : params(params),
      rng(rng),
      orderNodes(params.nbClients),
      orderRoutes(params.nbVehicles),
      lastModified(params.nbVehicles, -1)
{
    std::iota(orderNodes.begin(), orderNodes.end(), 1);
    std::iota(orderRoutes.begin(), orderRoutes.end(), 0);

    clients = std::vector<Node>(params.nbClients + 1);
    routes = std::vector<Route>(params.nbVehicles);
    startDepots = std::vector<Node>(params.nbVehicles);
    endDepots = std::vector<Node>(params.nbVehicles);

    for (int i = 0; i <= params.nbClients; i++)
    {
        clients[i].params = &params;
        clients[i].client = i;
    }

    for (int i = 0; i < params.nbVehicles; i++)
    {
        routes[i].params = &params;
        routes[i].idx = i;
        routes[i].depot = &startDepots[i];

        startDepots[i].params = &params;
        startDepots[i].client = 0;
        startDepots[i].route = &routes[i];

        startDepots[i].params = &params;
        endDepots[i].client = 0;
        endDepots[i].route = &routes[i];
    }
}
