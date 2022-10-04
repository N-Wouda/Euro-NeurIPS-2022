#include "crossover.h"
#include <unordered_map>
#include <set>

using Client = int;
using Clients = std::vector<Client>;
using Route = std::vector<Client>;
using Routes = std::vector<Route>;

/**
 * Currently works by turning the subtour into a tour directly
 * May still consider changing this into 2-opt as per Nagata et al. 2010
 */
std::vector<Client> resolveSubtour(std::vector<Client> &subtour,
    Params const &params){
    //selecting a specific client as the starting point results additional distance:
    //depot->tour[client] + tour[client-1]->depot - tour[client-1]->tour[client]

    int distance = params.dist(0,subtour.front()) + params.dist(subtour.back(), 0) 
                                    - params.dist(subtour.back(), subtour.front());
    Client best_client_index = 0;

    for(Client client = 1; client < static_cast<int>(subtour.size()); client++){
        int dist_alternative = params.dist(0,subtour[client]) + params.dist(subtour[client-1], 0)
                                    - params.dist(subtour[client-1],subtour[client]);
        if(dist_alternative < distance){
            distance = dist_alternative;
            best_client_index = client;
        }
    }

    //move clients from front to back until we start with the best client
    for(Client idx = 0; idx < best_client_index; idx++){
        subtour.push_back(subtour.front());
        subtour.erase(subtour.begin());
    }
    return subtour;
}


Individual edgeAssembly(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng)
{
    //Step 1:
    //Generate AB graph in XOR fashion
    /**
     * Unordered multimap as we can insert, access and delete through "from" node in constant time
     * and we can keep multiple elements with depot key
    **/
    std::unordered_multimap<Client, Client> AB_edges = {};
    std::unordered_multimap<Client, Client> A_edges = {};
    //edges included from parentB have their key shifted by an arbitrary amount so we can easily differentiate for step 2
    int shiftParentB = 2 * params.nbClients;

    //add all edges from depot
    std::set<Client> depotEdges = {};
    for (Route route : parents.first->getRoutes()){
        if(!route.empty()){
            A_edges.insert({0,route[1]});
            depotEdges.insert(route[1]);
        }
    }
    for (Route route : parents.second->getRoutes()){
        if(!route.empty()){
            if(depotEdges.count(route[1])){
                depotEdges.erase(route[1]);
            }else{
                depotEdges.insert(route[1]);
            }
        }
    }
    
    //now add all other edges
    auto const &neighboursA = parents.first->getNeighbours();
    auto const &neighboursB = parents.second->getNeighbours();

    for (Client client = 1; client <= params.nbClients; client++){
        // Test if client has a different successor in each individual
        if (neighboursA[client].second != neighboursB[client].second){
            //insert client from A
            AB_edges.insert({client, neighboursA[client].second});
            //insert client from B
            AB_edges.insert({client + shiftParentB, neighboursB[client].second});
        }
        A_edges.insert({client, neighboursA[client].second});
    }

    
    //Step 2:
    //Generate cycles based on a random node until all edges are gone
    //Based on Nagata et al. 2010 I believe it should deconstruct AB_edges into cycles, but not sure why
    std::vector<std::unordered_map<Client, Client>> AB_cycles = {};
    while(!AB_edges.empty()){
        std::unordered_map<Client, Client> cycle = {};
        auto edge = AB_edges.begin(); //technically selects an edge but we consider its start node only
        Client startNode = edge->first;
        bool isParentB = 0;
        if(startNode >= 2 * params.nbClients){
            startNode -= 2*params.nbClients;
            isParentB = 1;
        }
        Client currentNode = startNode;
        Client nextNode = startNode;
        
        do{
            edge = AB_edges.find(currentNode + shiftParentB * isParentB);
            nextNode = edge->second;
            cycle.insert({currentNode, nextNode});
            isParentB = !isParentB;
            currentNode = nextNode;
            AB_edges.erase(edge);
        }while(nextNode != startNode);

        AB_cycles.push_back(cycle);
    }

    //Step 3:
    //strategy = 1 is block based
    //strategy = 0 is single based
    std::unordered_map<Client, Client> *E_cycle;
    
    //TODO
    bool strategy = 0;

    if(strategy){

    }else{
        //pointer to element in vector? we don't mess with AB_cycles anymore so should be ok?
        E_cycle = &AB_cycles[rng.randint(AB_cycles.size())];
    }
    
    //Step 4:
    //remove intersect(E_A, E) and insert intersect(E_B, E)
    //1. loop over all nodes
    //2. find all edges e from client in E_A
    //3. if edge e in E as well, remove E[e]
    //4. if not, add E[e]
    //loop over all clients, find all edges from client in A, see if they are in E

    for(auto &edge : *E_cycle){
        //range as depot nodes may come up multiple times
        auto itpair = A_edges.equal_range(edge.first);
        bool edge_found = false;
        //if we find the node pair we erase it as it is in intersect(E_A,E)
        for(auto it = itpair.first; it != itpair.second; it++){
            if( it->second == edge.second ){
                A_edges.erase(it);
                edge_found = true;
                break;
            }
        }
        if(!edge_found){
            A_edges.insert({edge.first, edge.second});
        }
    }

    //We first reorder the maps into routes, all remaining nodes should be in subtour
    //TODO see if we can combine this with the previous loop
    Routes new_routes = {};
    auto depot_iterators = A_edges.equal_range(0);
    for(auto it = depot_iterators.first; it != depot_iterators.second; it++){
        new_routes.push_back({it->second});
    }
    A_edges.erase(depot_iterators.first, depot_iterators.second);
    for(auto &route : new_routes){
        //when route.back() == 0 we are back at depot
        while(route.back() != 0){
            route.push_back(A_edges.find(route.back())->second);
            A_edges.erase(route.back());
        }
        //remove depot from back again
        route.pop_back();
    }
    
    Routes subtours = {};
    while(!A_edges.empty()){
        std::pair<Client, Client> edge = *(A_edges.begin());
        subtours.push_back({});
        //if current subtour is not yet a cycle keep adding edges from A
        do{
            subtours.back().push_back(edge.first);
            A_edges.erase(edge.first);
            edge = *A_edges.find(edge.second);
        }while(subtours.back().front() != edge.second);
    }

    //Step 5:
    //Repair the routes
    //Currently we repair by just making subtours into real tours, 
    //i.e. we insert a depot somewhere in the subtour
    for(auto &subtour : subtours){
        new_routes.push_back(resolveSubtour(subtour, params));
    }
    return {&params, new_routes};
}

