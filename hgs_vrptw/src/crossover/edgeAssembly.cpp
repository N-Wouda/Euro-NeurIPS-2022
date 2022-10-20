#include "crossover.h"

using Client = int;
using Clients = std::vector<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;

struct ABGraph
{
    // successors for non-depot clients in A
    std::vector<Client> clientSuccA;
    // successors for depot in A
    std::vector<Client> depotSuccA;
    // predecessors for non-depot clients in B
    std::vector<Client> clientPredB;
    // predecessors for depot in B
    std::vector<Client> depotPredB;
    bool empty;
};

/**
 * Currently works by turning the subtour into a tour directly
 * May still consider changing this into 2-opt as per Nagata et al. 2010
 */
std::vector<Client> resolveSubtour(std::vector<Client> &subtour,
                                   Params const &params)
{
    // selecting a specific client as the starting point results additional
    // distance: depot->tour[client] + tour[client-1]->depot -
    // tour[client-1]->tour[client]

    int distance = params.dist(0, subtour.front())
                   + params.dist(subtour.back(), 0)
                   - params.dist(subtour.back(), subtour.front());
    size_t bestClientIdx = 0;

    for (size_t client = 1; client < subtour.size(); client++)
    {
        int distAlternative
            = params.dist(0, subtour[client])
              + params.dist(subtour[client - 1], 0)
              - params.dist(subtour[client - 1], subtour[client]);
        if (distAlternative < distance)
        {
            distance = distAlternative;
            bestClientIdx = client;
        }
    }

    // move clients from front to back until we start with the best client
    for (size_t idx = 0; idx < bestClientIdx; idx++)
    {
        subtour.push_back(subtour.front());
    }
    subtour.erase(subtour.begin(), subtour.begin() + bestClientIdx);
    return subtour;
}

// we do depth first search to find a AB cycle
// every recursion we switch parents,
bool findCycle(ABGraph &abGraph,
               Route &cycle,
               std::vector<Clients> &visited,
               bool selectFromA,
               Client current)
{
    Clients next = {-1};
    if (current == 0 && selectFromA)
    {
        // from depot and last parent was B
        if (!abGraph.depotSuccA.empty())
            next = abGraph.depotSuccA;
    }
    else if (current == 0 && !selectFromA)
    {
        // from depot and last parent was A
        if (!abGraph.depotPredB.empty())
            next = abGraph.depotPredB;
    }
    else if (selectFromA)
    {
        // not from depot and last parent was B
        next = {abGraph.clientSuccA[current]};
    }
    else
    {
        // not from depot and last parent was A
        next = {abGraph.clientPredB[current]};
    }
    for (auto &nextClient : next)
    {
        visited[selectFromA][current] = 1;

        if (nextClient == -1)
        {
            // we cannot continue from this node, return false
            return 0;
        }
        // we already got to this client, and used an edge of the opposite
        // parent -> AB-cycle complete.
        if (visited[!selectFromA][nextClient] == 1)
        {
            // we have found a cycle, append and return true
            cycle.push_back(nextClient);
            cycle.push_back(current);
            return 1;
        }

        bool cycleFound
            = findCycle(abGraph, cycle, visited, !selectFromA, nextClient);
        if (cycleFound)
        {
            cycle.push_back(current);
            return 1;
        }
    }
    return 0;
}

Individual
edgeAssembly(std::pair<Individual const *, Individual const *> const &parents,
             Params const &params,
             XorShift128 &rng)
{
    // Step 1:
    // We generate a graph from A and B in XOR fashion. (u,v) != (v,u)
    // Kept in two different objects as we need to differentiate between A and B
    // Also, edges from B are inverted i.e. e=(u,v) is denoted as e=(v,u)
    // vector of sets as we may have multiple edges from depots

    ABGraph abGraph;
    abGraph.clientSuccA.resize(params.nbClients + 1, -1);
    abGraph.clientPredB.resize(params.nbClients + 1, -1);
    abGraph.empty = 1;

    // fill all edges from A
    auto const &firstNeighbours = parents.first->getNeighbours();
    for (auto client = 1; client <= params.nbClients; client++)
    {
        auto const [pred, succ] = firstNeighbours[client];
        abGraph.clientSuccA[client] = succ;
        if (pred == 0)
        {
            abGraph.depotSuccA.push_back(client);
        }
    }

    // copy for later use
    Clients clientSuccChild = abGraph.clientSuccA;
    Clients depotSuccChild = abGraph.depotSuccA;

    // check if edge in B is already in A. Yes-> remove. No->invert and add to B
    auto const &secondNeighbours = parents.second->getNeighbours();
    for (auto client = 1; client <= params.nbClients; client++)
    {
        auto const [pred, succ] = secondNeighbours[client];

        // if edge is from depot search for it in A.
        if (pred == 0)
        {
            bool found = 0;
            for (auto it = abGraph.depotSuccA.begin();
                 it != abGraph.depotSuccA.end();
                 it++)
            {
                if (*it == client)
                {
                    abGraph.depotSuccA.erase(it);
                    found = 1;
                    break;
                }
            }
            if (!found)
            {
                abGraph.clientPredB[client] = 0;
                abGraph.empty = 0;
            }
        }

        // if [client,succ] in clientSuccA, replace by -1. Else, add [succ,
        // client] to clientPredB or depotPredB
        if (abGraph.clientSuccA[client] == succ)
        {
            abGraph.clientSuccA[client] = -1;
        }
        else
        {
            if (succ == 0)
            {
                abGraph.depotPredB.push_back(client);
            }
            else
            {
                abGraph.clientPredB[succ] = client;
            }
        }
    }

    // if parents are the same we stop
    if (abGraph.empty)
    {
        Routes some_parent = parents.first->getRoutes();
        return {&params, some_parent};
    }

    // Step 2:
    // We generate an AB-cycle if it exists and is connected to the random first
    // client that we select

    Route abCycle = {};
    std::vector<Clients> visited = {};
    Clients tmp(params.nbClients + 1, 0);
    visited.push_back(tmp);
    visited.push_back(tmp);

    Client firstClient = rng.randint(params.nbClients);
    bool hasCycle = findCycle(abGraph, abCycle, visited, 0, firstClient);
    if (!hasCycle)
    {
        // if we find no AB_cycles we just return the first parent
        Routes someParent = parents.first->getRoutes();
        return {&params, someParent};
    }
    // abCycle contains a cycle, but also a bunch of other nodes. Also, it is in
    // reverse order. thus, we look at uneven length subvectors until the first
    // node matches up with the last
    for (size_t idx = 0; idx < abCycle.size(); idx += 2)
    {
        if (abCycle[idx] == abCycle.front())
        {
            abCycle.resize(idx + 1);
            break;
        }
    }
    // get the correct order
    std::reverse(abCycle.begin(), abCycle.end());

    // Step 3: We only select a single cycle so no step 3.

    // Step 4:
    // remove intersect(E_A, E) and insert intersect(E_B, E)
    // 1. loop over ecycle
    // 2. check if in A -> yes: remove from A, no: insert in A
    // 3. if edge e in E as well, remove E[e]
    // 4. if not, add E[e]
    // loop over all clients, find all edges from client in A, see if they are
    // in E

    for (size_t idx = 0; idx < abCycle.size() - 1; idx++)
    {
        Client current;
        Client next;
        if (idx % 2 == 0)
        {
            // the edge is from A
            current = abCycle[idx];
            next = abCycle[idx + 1];
        }
        else
        {
            // the edge is from B
            current = abCycle[idx + 1];
            next = abCycle[idx];
        }
        if (current == 0)
        {
            for (auto it = depotSuccChild.rbegin(); it != depotSuccChild.rend();
                 it++)
            {
                if (*it == next)
                {
                    depotSuccChild.erase((it + 1).base());
                    break;
                }
                else
                {
                    depotSuccChild.push_back(next);
                    break;
                }
            }
        }
        else
        {
            if (clientSuccChild[current] == next)
            {
                clientSuccChild[current] = -1;
            }
            else
            {
                clientSuccChild[current] = next;
            }
        }
    }

    // We first reorder the maps into routes, all remaining nodes should be in
    // subtour
    Routes newRoutes = {};
    for (auto &el : depotSuccChild)
    {
        newRoutes.push_back({el});
    }
    for (auto &route : newRoutes)
    {
        Client current = route.back();
        // when route.back() == 0 we are back at depot
        Client next = clientSuccChild[current];
        while (next != 0)
        {
            route.push_back(next);
            current = next;
            next = clientSuccChild[current];
        }
    }
    for (auto &route : newRoutes)
    {
        for (auto &visited : route)
        {
            clientSuccChild[visited] = -1;
        }
    }

    Routes subtours = {};
    for (Client idx = 0; idx < static_cast<int>(clientSuccChild.size()); idx++)
    {
        if (clientSuccChild[idx] != -1)
        {
            Route subtour = {idx, clientSuccChild[idx]};
            Client current = subtour.back();
            Client next = clientSuccChild[current];

            while (next != subtour.front())
            {
                subtour.push_back(next);
                current = next;
                next = clientSuccChild[current];
            }

            for (auto &visited : subtour)
            {
                clientSuccChild[visited] = -1;
            }
        }
    }
    // Step 5:
    // Repair the routes
    // Currently we repair by just making subtours into real tours,
    // i.e. we insert a depot somewhere in the subtour
    for (auto &subtour : subtours)
    {
        newRoutes.push_back(resolveSubtour(subtour, params));
    }

    newRoutes.resize(params.nbVehicles);
    return {&params, newRoutes};
}
