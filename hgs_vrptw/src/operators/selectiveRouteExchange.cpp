#include "operators/crossover.h"

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <unordered_set>

namespace
{
using ClientSet = std::unordered_set<int>;
using Route = std::vector<int>;
using Routes = std::vector<Route>;

struct InsertPos  // best insert position, used to plan unplanned clients
{
    int deltaCost;
    Route *route;
    size_t offset;
};

// Evaluates the cost change of inserting client between prev and next.
inline int deltaCost(int client, int prev, int next, Params const &params)
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

// Uses a simple feasible nearest neighbour heuristic to insert unplanned
// clients into the given routes.
void addUnplannedToRoutes(ClientSet const &unplanned,
                          Routes &routes,
                          Params const &params)
{
    for (int client : unplanned)
    {
        InsertPos best = {INT_MAX, &routes.front(), 0};

        for (auto &route : routes)
        {
            if (route.empty())
                break;

            for (size_t idx = 0; idx <= route.size(); ++idx)
            {
                int prev, next;

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
                else  // Currently [idx - 1] -> [idx]. Try [idx - 1] -> client
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
    }
}
}  // namespace

Individual selectiveRouteExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng)
{
    // Get the number of routes of both parents
    size_t nOfRoutesA = parents.first->numRoutes();
    size_t nOfRoutesB = parents.second->numRoutes();

    auto const &routesA = parents.first->getRoutes();
    auto const &routesB = parents.second->getRoutes();

    // Picking the start index of routes to replace of parent A. We like to
    // replace routes with a large overlap of tasks, so we choose adjacent
    // routes (they are sorted on polar angle)
    size_t startA = rng.randint(static_cast<int>(nOfRoutesA));
    size_t nOfMovedRoutes
        = std::min(nOfRoutesA, nOfRoutesB) == 1  // prevent moving no routes
              ? 1
              : rng() % (std::min(nOfRoutesA - 1, nOfRoutesB - 1)) + 1;
    size_t startB = startA < nOfRoutesB ? startA : 0;

    ClientSet clientsInSelectedA;
    for (size_t r = 0; r < nOfMovedRoutes; r++)
        clientsInSelectedA.insert(routesA[(startA + r) % nOfRoutesA].begin(),
                                  routesA[(startA + r) % nOfRoutesA].end());

    ClientSet clientsInSelectedB;
    for (size_t r = 0; r < nOfMovedRoutes; r++)
        clientsInSelectedB.insert(routesB[(startB + r) % nOfRoutesB].begin(),
                                  routesB[(startB + r) % nOfRoutesB].end());

    while (true)
    {
        // Difference for moving 'left' in parent A
        int differenceALeft = 0;

        for (int c : routesA[(startA - 1 + nOfRoutesA) % nOfRoutesA])
            differenceALeft += !clientsInSelectedB.contains(c);

        for (int c : routesA[(startA + nOfMovedRoutes - 1) % nOfRoutesA])
            differenceALeft -= !clientsInSelectedB.contains(c);

        // Difference for moving 'right' in parent A
        int differenceARight = 0;

        for (int c : routesA[(startA + nOfMovedRoutes) % nOfRoutesA])
            differenceARight += !clientsInSelectedB.contains(c);

        for (int c : routesA[startA])
            differenceARight -= !clientsInSelectedB.contains(c);

        // Difference for moving 'left' in parent B
        int differenceBLeft = 0;

        for (int c : routesB[(startB - 1 + nOfMovedRoutes) % nOfRoutesB])
            differenceBLeft += clientsInSelectedA.contains(c);

        for (int c : routesB[(startB - 1 + nOfRoutesB) % nOfRoutesB])
            differenceBLeft -= clientsInSelectedA.contains(c);

        // Difference for moving 'right' in parent B
        int differenceBRight = 0;

        for (int c : routesB[startB])
            differenceBRight += clientsInSelectedA.contains(c);

        for (int c : routesB[(startB + nOfMovedRoutes) % nOfRoutesB])
            differenceBRight -= clientsInSelectedA.contains(c);

        int const bestDifference = std::min({differenceALeft,
                                             differenceARight,
                                             differenceBLeft,
                                             differenceBRight});

        if (bestDifference >= 0)  // there are no further improving moves
            break;

        if (bestDifference == differenceALeft)
        {
            for (int c : routesA[(startA + nOfMovedRoutes - 1) % nOfRoutesA])
                clientsInSelectedA.erase(clientsInSelectedA.find(c));

            startA = (startA - 1 + nOfRoutesA) % nOfRoutesA;

            for (int c : routesA[startA])
                clientsInSelectedA.insert(c);
        }
        else if (bestDifference == differenceARight)
        {
            for (int c : routesA[startA])
                clientsInSelectedA.erase(clientsInSelectedA.find(c));

            startA = (startA + 1) % nOfRoutesA;

            for (int c : routesA[(startA + nOfMovedRoutes - 1) % nOfRoutesA])
                clientsInSelectedA.insert(c);
        }
        else if (bestDifference == differenceBLeft)
        {
            for (int c : routesB[(startB + nOfMovedRoutes - 1) % nOfRoutesB])
                clientsInSelectedB.erase(clientsInSelectedB.find(c));

            startB = (startB - 1 + nOfRoutesB) % nOfRoutesB;

            for (int c : routesB[startB])
                clientsInSelectedB.insert(c);
        }
        else if (bestDifference == differenceBRight)
        {
            for (int c : routesB[startB])
                clientsInSelectedB.erase(clientsInSelectedB.find(c));

            startB = (startB + 1) % nOfRoutesB;
            for (int c : routesB[(startB + nOfMovedRoutes - 1) % nOfRoutesB])
                clientsInSelectedB.insert(c);
        }
    }

    // Identify differences between route sets
    ClientSet clientsInSelectedBNotA;
    for (int c : clientsInSelectedB)
        if (!clientsInSelectedA.contains(c))
            clientsInSelectedBNotA.insert(c);

    Routes routes1(params.nbVehicles);
    Routes routes2(params.nbVehicles);

    // Replace selected routes from parent A with routes from parent B
    for (size_t r = 0; r < nOfMovedRoutes; r++)
    {
        size_t indexA = (startA + r) % nOfRoutesA;
        size_t indexB = (startB + r) % nOfRoutesB;

        for (int c : routesB[indexB])
        {
            routes1[indexA].push_back(c);

            if (!clientsInSelectedBNotA.contains(c))
                routes2[indexA].push_back(c);
        }
    }

    // Move routes from parent A that are kept
    for (size_t r = nOfMovedRoutes; r < nOfRoutesA; r++)
    {
        size_t indexA = (startA + r) % nOfRoutesA;

        for (int c : routesA[indexA])
        {
            if (!clientsInSelectedBNotA.contains(c))
                routes1[indexA].push_back(c);

            routes2[indexA].push_back(c);
        }
    }

    // Insert unplanned clients (those that were in the removed routes of A, but
    // not the inserted routes of B).
    ClientSet unplanned;
    for (int c : clientsInSelectedA)
        if (!clientsInSelectedB.contains(c))
            unplanned.insert(c);

    addUnplannedToRoutes(unplanned, routes1, params);
    addUnplannedToRoutes(unplanned, routes2, params);

    Individual indiv1{&params, parents.first->getTour(), routes1};
    Individual indiv2{&params, parents.second->getTour(), routes2};

    return indiv1.cost() < indiv2.cost() ? indiv1 : indiv2;
}
