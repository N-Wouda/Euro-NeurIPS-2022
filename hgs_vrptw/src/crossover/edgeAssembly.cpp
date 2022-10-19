#include "crossover.h"

using Client = int;
using Clients = std::vector<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;

struct AB_graph
{
    std::vector<Client> A;
    std::vector<Client> B;
    std::vector<Client> A_from_depot;
    std::vector<Client> B_to_depot;
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
    Client best_client_index = 0;

    for (Client client = 1; client < static_cast<int>(subtour.size()); client++)
    {
        int dist_alternative
            = params.dist(0, subtour[client])
              + params.dist(subtour[client - 1], 0)
              - params.dist(subtour[client - 1], subtour[client]);
        if (dist_alternative < distance)
        {
            distance = dist_alternative;
            best_client_index = client;
        }
    }

    // move clients from front to back until we start with the best client
    for (Client idx = 0; idx < best_client_index; idx++)
    {
        subtour.push_back(subtour.front());
        subtour.erase(subtour.begin());
    }
    return subtour;
}

// we do depth first search to find a cycle
bool find_cycle(AB_graph &ab_graph,
                Route &cycle,
                std::vector<Clients> &visited,
                bool parent,
                Client current)
{
    Clients next = {-1};
    if (current == 0 && parent)
    {
        // from depot and last parent was B
        if (!ab_graph.A_from_depot.empty())
            next = ab_graph.A_from_depot;
    }
    else if (current == 0 && !parent)
    {
        // from depot and last parent was A
        if (!ab_graph.B_to_depot.empty())
            next = ab_graph.B_to_depot;
    }
    else if (parent)
    {
        // not from depot and last parent was B
        next = {ab_graph.A[current]};
    }
    else
    {
        // not from depot and last parent was A
        next = {ab_graph.B[current]};
    }
    for (auto &next_client : next)
    {
        visited[!parent][current] = 1;

        if (next_client == -1)
        {
            // we cannot continue from this node, return false
            return 0;
        }

        if (visited[parent][next_client] == 1)
        {
            // we have found a cycle, append and return true
            cycle.push_back(next_client);
            cycle.push_back(current);
            return 1;
        }

        bool cycleFound
            = find_cycle(ab_graph, cycle, visited, !parent, next_client);
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

    AB_graph ab_graph;
    ab_graph.A.resize(params.nbClients + 1, -1);
    ab_graph.B.resize(params.nbClients + 1, -1);
    ab_graph.empty = 1;

    // fill all edges from A
    for (Route route : parents.first->getRoutes())
    {
        if (!route.empty())
        {
            ab_graph.A_from_depot.push_back(route.front());
            for (Client idx = 1; idx < static_cast<int>(route.size()); idx++)
            {
                ab_graph.A[route[idx - 1]] = route[idx];
            }
            ab_graph.A[route.back()] = 0;
        }
    }
    // copy for later use
    Clients child_edges = ab_graph.A;
    Clients child_from_depot = ab_graph.A_from_depot;

    // check if edge in B is already in A. Yes-> remove. No->invert and add to B
    for (Route route : parents.second->getRoutes())
    {
        if (!route.empty())
        {
            // first edge from depot
            bool found = 0;
            for (auto it = ab_graph.A_from_depot.begin();
                 it != ab_graph.A_from_depot.end();
                 it++)
            {
                if (*it == route.front())
                {
                    ab_graph.A_from_depot.erase(it);
                    found = 1;
                    break;
                }
            }
            if (!found)
            {
                ab_graph.B[route.front()] = 0;
                ab_graph.empty = 0;
            }
            // clients in middle of route
            for (Client idx = 1; idx < static_cast<int>(route.size()); idx++)
            {
                if (ab_graph.A[route[idx - 1]] == route[idx])
                {
                    ab_graph.A[route[idx - 1]] = -1;
                }
                else
                {
                    ab_graph.B[route[idx]] = route[idx - 1];
                    ab_graph.empty = 0;
                }
            }
            // last edge to depot
            if (ab_graph.A[route.back()] == 0)
            {
                ab_graph.A[route.back()] = -1;
            }
            else
            {
                ab_graph.B_to_depot.push_back(route.back());
                ab_graph.empty = 0;
            }
        }
    }

    // if parents are the same we stop
    if (ab_graph.empty)
    {
        Routes some_parent = parents.first->getRoutes();
        return {&params, some_parent};
    }

    // Step 2:
    // Generate cycles based on a random node until all edges are gone
    // Based on Nagata et al. 2010 I believe it should deconstruct AB_edges into
    // cycles, but not sure why

    Route AB_cycle = {};
    std::vector<Clients> visited = {};
    Clients tmp(params.nbClients + 1, 0);
    visited.push_back(tmp);
    visited.push_back(tmp);

    Client first_client = rng.randint(params.nbClients);
    bool succes = find_cycle(ab_graph, AB_cycle, visited, 0, first_client);
    if (!succes)
    {
        // if we find no AB_cycles we just return the first parent
        Routes some_parent = parents.first->getRoutes();
        return {&params, some_parent};
    }

    for (size_t idx = 0; idx < AB_cycle.size(); idx++)
    {
        if (idx % 2 == 0 && AB_cycle[idx] == AB_cycle.front())
        {
            AB_cycle.resize(idx + 1);
            break;
        }
    }
    std::reverse(AB_cycle.begin(), AB_cycle.end());

    // Step 3: We only select a single cycle so no step 3.

    // Step 4:
    // remove intersect(E_A, E) and insert intersect(E_B, E)
    // 1. loop over ecycle
    // 2. check if in A -> yes: remove from A, no: insert in A
    // 3. if edge e in E as well, remove E[e]
    // 4. if not, add E[e]
    // loop over all clients, find all edges from client in A, see if they are
    // in E

    for (size_t idx = 0; idx < AB_cycle.size() - 1; idx++)
    {
        Client current;
        Client next;
        if (idx % 2 == 0)
        {
            // the edge is from A
            current = AB_cycle[idx];
            next = AB_cycle[idx + 1];
        }
        else
        {
            // the edge is from B
            current = AB_cycle[idx + 1];
            next = AB_cycle[idx];
        }
        if (current == 0)
        {
            for (auto it = child_from_depot.end();
                 it != child_from_depot.begin();)
            {
                it--;
                if (*it == next)
                {
                    child_from_depot.erase(it);
                    break;
                }
                else
                {
                    child_from_depot.push_back(next);
                    break;
                }
            }
        }
        else
        {
            if (child_edges[current] == next)
            {
                child_edges[current] = -1;
            }
            else
            {
                child_edges[current] = next;
            }
        }
    }

    // We first reorder the maps into routes, all remaining nodes should be in
    // subtour
    Routes new_routes = {};
    for (auto &el : child_from_depot)
    {
        new_routes.push_back({el});
    }
    for (auto &route : new_routes)
    {
        Client current = route.back();
        // when route.back() == 0 we are back at depot
        Client next = child_edges[current];
        while (next != 0)
        {
            route.push_back(next);
            current = next;
            next = child_edges[current];
        }
    }
    for (auto &route : new_routes)
    {
        for (auto &visited : route)
        {
            child_edges[visited] = -1;
        }
    }

    Routes subtours = {};
    for (Client idx = 0; idx < static_cast<int>(child_edges.size()); idx++)
    {
        if (child_edges[idx] != -1)
        {
            Route subtour = {idx, child_edges[idx]};
            Client current = subtour.back();
            Client next = child_edges[current];

            while (next != subtour.front())
            {
                subtour.push_back(next);
                current = next;
                next = child_edges[current];
            }

            for (auto &visited : subtour)
            {
                child_edges[visited] = -1;
            }
        }
    }
    // Step 5:
    // Repair the routes
    // Currently we repair by just making subtours into real tours,
    // i.e. we insert a depot somewhere in the subtour
    for (auto &subtour : subtours)
    {
        new_routes.push_back(resolveSubtour(subtour, params));
    }

    new_routes.resize(params.nbVehicles);
    return {&params, new_routes};
}
