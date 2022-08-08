#include "LocalSearch.h"

#include "Individual.h"
#include "Params.h"

#include <numeric>
#include <stdexcept>
#include <vector>

void LocalSearch::operator()(Individual &indiv,
                             int excessCapacityPenalty,
                             int timeWarpPenalty)
{
    penalties = {&params, excessCapacityPenalty, timeWarpPenalty};

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

    bool const shouldIntensify
        = rng.randint(100) < (unsigned)params.config.intensificationProbability;

    searchCompleted = false;
    nbMoves = 0;

    for (int step = 0; !searchCompleted; ++step)
    {
        if (step > 1)                // At least two loops as some moves with
            searchCompleted = true;  // empty routes are not done in the first

        /* ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY RESTRICTION */
        for (int const u : orderNodes)
        {
            Node *nodeU = &clients[u];
            int lastTestRINodeU = nodeU->whenLastTestedRI;
            nodeU->whenLastTestedRI = nbMoves;

            // Randomizing the order of the neighborhoods within this loop does
            // not matter much as we are already randomizing the order of the
            // node pairs (and it's not very common to find improving moves of
            // different types for the same node pair)
            for (auto const v : params.getNeighboursOf(nodeU->client))
            {
                Node *nodeV = &clients[v];
                auto const lastTested
                    = std::max(nodeU->route->whenLastModified,
                               nodeV->route->whenLastModified);

                // only evaluate moves involving routes that have been modified
                // since last move evaluations for nodeU
                if (step == 0 || lastTested > lastTestRINodeU)
                {
                    if (applyNodeOperators(nodeU, nodeV))
                        continue;

                    // Trying moves that insert nodeU directly after the depot
                    if (nodeV->prev->isDepot()
                        && applyNodeOperators(nodeU, nodeV->prev))
                        continue;
                }
            }

            /* MOVES INVOLVING AN EMPTY ROUTE -- NOT TESTED IN THE FIRST LOOP TO
             * AVOID INCREASING TOO MUCH THE FLEET SIZE */
            if (step > 0)
            {
                auto pred = [](auto const &route) { return route.empty(); };
                auto empty = std::find_if(routes.begin(), routes.end(), pred);

                if (empty == routes.end())
                    continue;

                if (applyNodeOperators(nodeU, empty->depot))
                    continue;
            }
        }

        /* (SWAP*) MOVES LIMITED TO ROUTE PAIRS WHOSE CIRCLE SECTORS OVERLAP */
        if (searchCompleted && shouldIntensify)
        {
            for (int const rU : orderRoutes)
            {
                Route &routeU = routes[rU];

                if (routeU.empty())
                    continue;

                int lastTestLargeNbRouteU = routeU.whenLastTestedLargeNb;
                routeU.whenLastTestedLargeNb = nbMoves;

                for (int const rV : orderRoutes)
                {
                    Route &routeV = routes[rV];

                    if (routeV.empty() || routeU.idx >= routeV.idx)
                        continue;

                    if (step > 0
                        && std::max(routeU.whenLastModified,
                                    routeV.whenLastModified)
                               <= lastTestLargeNbRouteU)
                        continue;

                    if (!routeU.overlapsWith(routeV))
                        continue;

                    if (applyRouteOperators(&routeU, &routeV))
                        continue;
                }
            }
        }
    }
}

bool LocalSearch::applyNodeOperators(Node *U, Node *V)
{
    for (auto const &op : nodeOps)
    {
        auto *routeU = U->route;  // copy these because the operator could
        auto *routeV = V->route;  // modify the node's route membership

        if (op(U, V, penalties))
        {
            nbMoves++;
            searchCompleted = false;

            routeU->update(nbMoves, penalties);
            if (routeU != routeV)
                routeV->update(nbMoves, penalties);

            return true;
        }
    }

    return false;
}

bool LocalSearch::applyRouteOperators(Route *U, Route *V)
{
    for (auto const &op : routeOps)
        if (op(U, V, penalties))
        {
            nbMoves++;
            searchCompleted = false;

            U->update(nbMoves, penalties);
            if (U != V)
                V->update(nbMoves, penalties);

            return true;
        }

    return false;
}

void LocalSearch::loadIndividual(Individual const &indiv)
{
    for (int client = 0; client <= params.nbClients; client++)
    {
        clients[client].whenLastTestedRI = -1;
        clients[client].tw = {&params,
                              client,
                              client,
                              params.clients[client].servDur,
                              0,
                              params.clients[client].twEarly,
                              params.clients[client].twLate,
                              params.clients[client].releaseTime};
    }

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

        route->update(0, penalties);
        route->whenLastTestedLargeNb = -1;
    }
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
    : penalties{&params, params.penaltyCapacity, params.penaltyTimeWarp},
      params(params),
      rng(rng),
      orderNodes(params.nbClients),
      orderRoutes(params.nbVehicles)
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

        startDepots[i].client = 0;
        startDepots[i].route = &routes[i];

        endDepots[i].client = 0;
        endDepots[i].route = &routes[i];
    }
}
