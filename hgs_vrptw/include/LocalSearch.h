#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include "CircleSector.h"
#include "Individual.h"
#include "Node.h"
#include "Params.h"
#include "Penalties.h"
#include "Route.h"
#include "TimeWindowSegment.h"
#include "XorShift128.h"

#include <array>
#include <functional>
#include <set>
#include <vector>

class LocalSearch
{
    using nodeOp = std::function<bool(Node *, Node *, Penalties const &)>;
    using routeOp = std::function<bool(Route *, Route *, Penalties const &)>;

    Penalties penalties;  // Penalty data
    Params &params;       // Problem parameters
    XorShift128 &rng;     // Random number generator

    std::vector<int> orderNodes;   // random node order used in RI operators
    std::vector<int> orderRoutes;  // random route order used in SWAP* operators

    /* THE SOLUTION IS REPRESENTED AS A LINKED LIST OF ELEMENTS */
    std::vector<Node> clients;    // Note that clients[0] is a sentinel value
    std::vector<Node> depots;     // These depots mark the start of routes
    std::vector<Node> depotsEnd;  // These depots mark the end of routes
    std::vector<Route> routes;

    std::vector<nodeOp> nodeOps;
    std::vector<routeOp> routeOps;

    int nbMoves = 0;               // Operator (RI and SWAP*) counter
    bool searchCompleted = false;  // No further improving move?

    // Load an initial solution that we will attempt to improve
    void loadIndividual(Individual const &indiv);

    // Export the LS solution back into an individual
    Individual exportIndividual();

    bool applyNodeOperators(Node *U, Node *V);

    bool applyRouteOperators(Route *U, Route *V);

    // Performs the actual local search procedure
    void search();

public:
    /**
     * Adds a local search operator that works on node/client pairs U and V.
     */
    void addNodeOperator(nodeOp const &op) { nodeOps.push_back(op); }

    /**
     * Adds a local search operator that works on route pairs U and V. These
     * operators are executed for route pairs whose circle sectors overlap.
     */
    void addRouteOperator(routeOp const &op) { routeOps.push_back(op); }

    /**
     * Performs the local search procedure around the given individual, using
     * the passed-in penalty parameters.
     *
     * @param indiv                  Individual to improve.
     * @param excessCapacityPenalty  Excess capacity penalty.
     * @param timeWarpPenalty        Penalty for violated time windows.
     */
    void operator()(Individual &indiv,
                    int excessCapacityPenalty,
                    int timeWarpPenalty);

    LocalSearch(Params &params, XorShift128 &rng);
};

#endif
