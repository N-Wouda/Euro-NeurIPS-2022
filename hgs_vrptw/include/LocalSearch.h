#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include "Individual.h"
#include "Node.h"
#include "Params.h"
#include "Penalties.h"
#include "Route.h"
#include "XorShift128.h"

#include "LocalSearchOperator.h"

#include <functional>
#include <vector>

class LocalSearch
{
    using NodeOp = LocalSearchOperator<Node>;
    using RouteOp = LocalSearchOperator<Route>;

    Penalties penalties;  // Penalty data
    Params &params;       // Problem parameters
    XorShift128 &rng;     // Random number generator

    std::vector<int> orderNodes;   // random node order used in RI operators
    std::vector<int> orderRoutes;  // random route order used in SWAP* operators

    std::vector<int> lastModified;  // tracks when routes were last modified

    std::vector<Node> clients;      // Note that clients[0] is a sentinel value
    std::vector<Node> startDepots;  // These mark the start of routes
    std::vector<Node> endDepots;    // These mark the end of routes
    std::vector<Route> routes;

    std::vector<NodeOp *> nodeOps;
    std::vector<RouteOp *> routeOps;

    int nbMoves = 0;               // Operator (RI and SWAP*) counter
    bool searchCompleted = false;  // No further improving move found?

    // Load an initial solution that we will attempt to improve
    void loadIndividual(Individual const &indiv);

    // Export the LS solution back into an individual
    Individual exportIndividual();

    [[nodiscard]] bool applyNodeOperators(Node *U, Node *V);

    [[nodiscard]] bool applyRouteOperators(Route *U, Route *V);

    // Performs the actual local search procedure
    void search();

public:
    /**
     * Adds a local search operator that works on node/client pairs U and V.
     */
    void addNodeOperator(NodeOp &op) { nodeOps.emplace_back(&op); }

    /**
     * Adds a local search operator that works on route pairs U and V. These
     * operators are executed for route pairs whose circle sectors overlap.
     */
    void addRouteOperator(RouteOp &op) { routeOps.emplace_back(&op); }

    /**
     * Performs the local search procedure around the given individual, using
     * the passed-in penalty parameters.
     *
     * @param indiv           Individual to improve.
     * @param loadPenalty     Excess load penalty.
     * @param timeWarpPenalty Penalty for violated time windows.
     */
    void operator()(Individual &indiv, int loadPenalty, int timeWarpPenalty);

    LocalSearch(Params &params, XorShift128 &rng);
};

#endif
