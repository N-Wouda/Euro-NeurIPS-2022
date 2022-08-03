#include "crossover.h"

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_set>

namespace
{
using Parents = std::pair<Individual const *, Individual const *>;
using Client = int;
using ClientSet = std::unordered_set<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;
using RouteSet = std::vector<Route>;  // TODO Make an unordered_set
using Destroyed = std::pair<Routes, ClientSet>;

struct InsertPos  // best insert position, used to plan unplanned clients
{
    int deltaCost;
    Route *route;
    size_t offset;
};

void printRouteSize(Routes const routes, std::string text)
{
    // std::cout << text;
    // int tot = 0;
    // for (auto &route : routes)
    //     tot += route.size();
    // printf("%d\n", tot);
}

void printClientSet(ClientSet const route, std::string text)
{
    // std::cout << text;
    // for (auto c : route)
    //     printf("%d-", c);
    // printf("\n");
}

void printRoute(Route const route, std::string text)
{
    // std::cout << text;
    // for (auto c : route)
    //     printf("%d-", c);
    // printf("\n");
}
Destroyed stringRemoval(Routes routes,
                        Client center,
                        Params const &params,
                        XorShift128 &rng)
{
    // Compute the maximum string cardinality to be removed
    size_t avgRouteSize = 0;
    for (auto &route : routes)
    {
        avgRouteSize += route.size();
    }
    avgRouteSize = avgRouteSize / routes.size();
    auto maxCard = std::min(params.config.maxStringCard, avgRouteSize);

    // Compute the number of strings to remove
    size_t nbStringRemovals
        = rng.randint(
              std::max(static_cast<size_t>(1),
                       (4 * params.config.avgDestruction) / (1 + maxCard) - 1))
          + 1;

    RouteSet destroyedRoutes;
    ClientSet removedClients;

    for (auto client : params.getNeighboursOf(center))
    {

        if (destroyedRoutes.size() >= nbStringRemovals)
            break;

        if (removedClients.contains(client))
            continue;

        for (auto &route : routes)
        {
            if (std::find(route.begin(), route.end(), client) != route.end())
            {
                if (std::find(
                        destroyedRoutes.begin(), destroyedRoutes.end(), route)
                    != destroyedRoutes.end())
                    continue;

                // Remove string from the route
                auto card = rng.randint(std::min(
                                static_cast<size_t>(route.size()), maxCard))
                            + 1;

                std::vector<int>::iterator itr
                    = std::find(route.begin(), route.end(), client);
                int idx = std::distance(route.begin(), itr);
                auto pos = rng.randint(card);
                auto start = idx - pos;

                std::vector<int> idcs;
                for (auto i = start; i < start + card; i++)
                    idcs.push_back(i % route.size());

                // printRoute(route, std::string("Old route: "));

                // printf("String indices: ");
                // for (auto i : idcs)
                //     printf("%d-", i);
                // printf("\n");

                // printf("Route string: ");
                // for (auto i : idcs)
                //     printf("%d-", route[i]);
                // printf("\n");

                ClientSet removed;
                for (auto idx : idcs)
                    removed.insert(route[idx]);

                // TODO Hoe do I remove these clients?
                for (auto c : removed)
                {
                    std::vector<int>::iterator position
                        = std::find(route.begin(), route.end(), c);
                    route.erase(position);
                }

                // printRoute(route, std::string("New route: "));

                // if (rng.randint(0) <= params.config.splitRate)
                // auto idcs = selectString(currCurrRoute, client, card, rng);

                // // TODO split string

                destroyedRoutes.push_back(route);
                for (auto c : removed)
                    removedClients.insert(c);
                break;
            }
        }

        // printf("Removed clients: %d\n", removedClients.size());
        // printRouteSize(routes, std::string("Size just before returning: "));
        return std::make_pair(routes, removedClients);
    }
}

int deltaCost(Client client, Client prev, Client next, Params const &params)
{
    int clientLate = params.clients[client].twLate;
    int distToInsert = params.dist(prev, client);
    int prevEarly = params.clients[prev].twEarly;

    if (prevEarly + distToInsert >= clientLate)
        return INT_MAX;

    int clientEarly = params.clients[client].twEarly;
    int distFromInsert = params.dist(client, next);
    int nextLate = params.clients[next].twLate;

    if (clientEarly + distFromInsert >= nextLate)
        return INT_MAX;

    return distToInsert + distFromInsert - params.dist(prev, next);
}

Individual greedyRepairWithBlinks(Routes &routes,
                                  ClientSet unplanned,
                                  Params const &params,
                                  XorShift128 &rng)

{
    printRouteSize(routes, std::string("Initial: "));
    // TODO sort clients
    for (Client client : unplanned)
    {
        InsertPos best = {INT_MAX, &routes.front(), 0};

        for (auto &route : routes)
        {
            if (route.empty())
                break;

            for (size_t idx = 0; idx <= route.size(); ++idx)
            {
                // TODO add blinks
                Client prev, next;

                if (idx == 0)  // Currently depot -> [0]. Try depot -> client
                {              // -> [0].
                    prev = 0;
                    next = route[0];
                }
                else if (idx == route.size())  // Currently [-1] -> depot. Try
                {                              // [-1] -> client -> depot.
                    prev = route.back();
                    next = 0;
                }
                else  // Currently [idx - 1] -> [idx]. Try [idx - 1] ->
                      // client
                {     // -> [idx].
                    prev = route[idx - 1];
                    next = route[idx];
                }

                int const cost = deltaCost(client, prev, next, params);
                if (cost < best.deltaCost)
                    best = {cost, &route, idx};
            }
        }

        auto const [_, route, offset] = best;
        route->insert(route->begin() + static_cast<long>(offset), client);
        printRouteSize(routes, std::string("After:"));
    }
    return {&params, routes};
}
}  // namespace

Individual stringRemovalExchange(Parents const &parents,
                                 Params const &params,
                                 XorShift128 &rng)
{
    auto const &routes1 = parents.first->getRoutes();
    auto const &routes2 = parents.second->getRoutes();

    // Find a center node around which substrings will be removed
    Client center = rng.randint(params.nbClients) + 1;

    Destroyed destroyed1 = stringRemoval(routes1, center, params, rng);
    Destroyed destroyed2 = stringRemoval(routes2, center, params, rng);

    // Remove clients from other destroyed parent
    for (Client c : destroyed2.second)
    {
        for (auto &route : destroyed1.first)
        {
            if (std::find(route.begin(), route.end(), c) != route.end())
            {
                std::vector<int>::iterator position
                    = std::find(route.begin(), route.end(), c);
                route.erase(position);
                break;
            }
        }
    }

    for (Client c : destroyed1.second)
    {
        for (auto &route : destroyed2.first)
        {
            if (std::find(route.begin(), route.end(), c) != route.end())
            {
                std::vector<int>::iterator position
                    = std::find(route.begin(), route.end(), c);
                route.erase(position);
                break;
            }
        }
    }

    ClientSet removed;
    for (Client c : destroyed1.second)
        removed.insert(c);
    for (Client c : destroyed2.second)
        removed.insert(c);

    Individual indiv1
        = greedyRepairWithBlinks(destroyed1.first, removed, params, rng);
    Individual indiv2
        = greedyRepairWithBlinks(destroyed2.first, removed, params, rng);

    return std::min(indiv1, indiv2);
}
