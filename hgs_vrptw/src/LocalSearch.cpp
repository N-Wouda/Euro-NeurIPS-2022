#include "LocalSearch.h"

#include "Individual.h"
#include "Params.h"
#include "TimeWindowSegment.h"

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
                    if (nodeV->prev->isDepot
                        && applyNodeOperators(nodeU, nodeV->prev))
                        continue;
                }
            }

            /* MOVES INVOLVING AN EMPTY ROUTE -- NOT TESTED IN THE FIRST LOOP TO
             * AVOID INCREASING TOO MUCH THE FLEET SIZE */
            if (step > 0)
            {
                auto pred = [](auto &route) { return route.nbCustomers == 0; };
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
            for (int const rU: orderRoutes)
            {
                Route &routeU = routes[rU];
                
                if (routeU.nbCustomers == 0)
                    continue;

                int lastTestLargeNbRouteU = routeU.whenLastTestedLargeNb;
                routeU.whenLastTestedLargeNb = nbMoves;

                for (int const rV: orderRoutes)
                {
                    Route &routeV = routes[rV];

                    if (routeV.nbCustomers == 0 || routeU.idx >= routeV.idx)
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
    TimeWindowSegment depotTwData = {&params,
                                     0,
                                     0,
                                     0,
                                     0,
                                     params.clients[0].twEarly,
                                     params.clients[0].twLate,
                                     params.clients[0].releaseTime};

    // Initializing time window data (before loop since it is needed in update
    // route)
    for (int i = 1; i <= params.nbClients; i++)
    {
        clients[i].tw = {&params,
                         i,
                         i,
                         params.clients[i].servDur,
                         0,
                         params.clients[i].twEarly,
                         params.clients[i].twLate,
                         params.clients[i].releaseTime};
    }

    auto const &routesIndiv = indiv.getRoutes();

    for (int r = 0; r < params.nbVehicles; r++)
    {
        Node *myDepot = &depots[r];
        Node *myDepotFin = &depotsEnd[r];
        Route *myRoute = &routes[r];
        myDepot->prev = myDepotFin;
        myDepotFin->next = myDepot;
        if (!routesIndiv[r].empty())
        {
            Node *myClient = &clients[routesIndiv[r][0]];
            myClient->route = myRoute;
            myClient->prev = myDepot;
            myDepot->next = myClient;
            for (int i = 1; i < static_cast<int>(routesIndiv[r].size()); i++)
            {
                Node *myClientPred = myClient;
                myClient = &clients[routesIndiv[r][i]];
                myClient->prev = myClientPred;
                myClientPred->next = myClient;
                myClient->route = myRoute;
            }
            myClient->next = myDepotFin;
            myDepotFin->prev = myClient;
        }
        else
        {
            myDepot->next = myDepotFin;
            myDepotFin->prev = myDepot;
        }

        myDepot->tw = depotTwData;
        myDepot->twBefore = depotTwData;
        myDepot->twAfter = depotTwData;
        myDepot->isSeed = false;

        myDepotFin->tw = depotTwData;
        myDepotFin->twBefore = depotTwData;
        myDepotFin->twAfter = depotTwData;
        myDepotFin->isSeed = false;

        routes[r].update(0, penalties);
        routes[r].whenLastTestedLargeNb = -1;
    }

    for (int i = 1; i <= params.nbClients; i++)
        clients[i].whenLastTestedRI = -1;
}

Individual LocalSearch::exportIndividual()
{
    std::vector<std::pair<double, int>> routePolarAngles;
    routePolarAngles.reserve(params.nbVehicles);

    for (int r = 0; r < params.nbVehicles; r++)
        routePolarAngles.emplace_back(routes[r].polarAngleBarycenter, r);

    // empty indivRoutes have a polar angle of 1.e30, and therefore will always
    // appear at the end
    std::sort(routePolarAngles.begin(), routePolarAngles.end());

    std::vector<std::vector<int>> indivRoutes(params.nbVehicles);

    for (int r = 0; r < params.nbVehicles; r++)
    {
        Node *node = depots[routePolarAngles[r].second].next;

        while (!node->isDepot)
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
    clients = std::vector<Node>(params.nbClients + 1);
    routes = std::vector<Route>(params.nbVehicles);
    depots = std::vector<Node>(params.nbVehicles);
    depotsEnd = std::vector<Node>(params.nbVehicles);

    for (int i = 0; i <= params.nbClients; i++)
    {
        clients[i].params = &params;
        clients[i].client = i;
        clients[i].isDepot = false;
    }

    for (int i = 0; i < params.nbVehicles; i++)
    {
        routes[i].params = &params;
        routes[i].idx = i;
        routes[i].depot = &depots[i];
        depots[i].client = 0;
        depots[i].isDepot = true;
        depots[i].route = &routes[i];
        depotsEnd[i].client = 0;
        depotsEnd[i].isDepot = true;
        depotsEnd[i].route = &routes[i];
    }

    std::iota(orderNodes.begin(), orderNodes.end(), 1);
    std::iota(orderRoutes.begin(), orderRoutes.end(), 0);
}
