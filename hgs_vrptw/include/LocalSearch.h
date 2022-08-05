#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include "CircleSector.h"
#include "Individual.h"
#include "Node.h"
#include "Params.h"
#include "Route.h"
#include "TimeWindowSegment.h"
#include "XorShift128.h"

#include <array>
#include <set>
#include <vector>

class LocalSearch
{
public:
    struct Penalties
    {
        Params const *params;
        int loadPenalty;
        int timePenalty;

        // Computes the total excess capacity penalty for the given load
        [[nodiscard]] inline int load(int currLoad) const
        {
            auto const excessLoad = currLoad - params->vehicleCapacity;
            return std::max(excessLoad, 0) * loadPenalty;
        }

        // Computes the total time warp penalty for the given time window data
        [[nodiscard]] inline int timeWarp(TimeWindowSegment const &twData) const
        {
            return twData.totalTimeWarp() * timePenalty;
        }
    };

private:
    // Structure used in SWAP* to remember the three best insertion positions of
    // a client in a given route
    struct ThreeBestInsert
    {
        int whenLastCalculated = 0;
        std::array<int, 3> bestCost = {INT_MAX, INT_MAX, INT_MAX};
        std::array<Node *, 3> bestLocation = {nullptr, nullptr, nullptr};

        void add(int costInsert, Node *placeInsert)
        {
            if (costInsert >= bestCost[2])
                return;

            if (costInsert >= bestCost[1])
            {
                bestCost[2] = costInsert;
                bestLocation[2] = placeInsert;
            }
            else if (costInsert >= bestCost[0])
            {
                bestCost[2] = bestCost[1];
                bestLocation[2] = bestLocation[1];
                bestCost[1] = costInsert;
                bestLocation[1] = placeInsert;
            }
            else
            {
                bestCost[2] = bestCost[1];
                bestLocation[2] = bestLocation[1];
                bestCost[1] = bestCost[0];
                bestLocation[1] = bestLocation[0];
                bestCost[0] = costInsert;
                bestLocation[0] = placeInsert;
            }
        }
    };

    // Structure used to keep track of the best SWAP* move
    struct SwapStarElement
    {
        int moveCost = INT_MAX;
        int loadPenU = INT_MAX;
        int loadPenV = INT_MAX;
        Node *U = nullptr;
        Node *bestPositionU = nullptr;
        Node *V = nullptr;
        Node *bestPositionV = nullptr;
    };

    Penalties penalties;
    Params &params;    // Problem parameters
    XorShift128 &rng;  // Random number generator

    std::vector<int> orderNodes;   // random node order used in RI operators
    std::vector<int> orderRoutes;  // random route order used in SWAP* operators

    /* THE SOLUTION IS REPRESENTED AS A LINKED LIST OF ELEMENTS */
    std::vector<Node> clients;    // Note that clients[0] is a sentinel value
    std::vector<Node> depots;     // These depots mark the start of routes
    std::vector<Node> depotsEnd;  // These depots mark the end of routes
    std::vector<Route> routes;
    std::vector<bool> bestInsertInitializedForRoute;
    std::vector<std::vector<ThreeBestInsert>>
        bestInsertClient;  // (SWAP*) For each route and node, storing the
                           // cheapest insertion cost (excluding TW)
    std::vector<std::vector<ThreeBestInsert>>
        bestInsertClientTW;  // (SWAP*) For each route and node, storing the
                             // cheapest insertion cost (including TW)

    /* SUB-ROUTINES FOR EFFICIENT SWAP* EVALUATIONS */

    // Calculates all SWAP* between routeU and routeV and apply the best
    // improving move
    bool swapStar(bool withTW,
                  int &nbMoves,
                  bool &searchCompleted,
                  Route *routeU,
                  Route *routeV);

    // Calculates the insertion cost and position in the route of V, where V is
    // omitted
    int getCheapestInsertSimultRemoval(Node *U, Node *V, Node *&bestPosition);

    // Calculates the insertion cost and position in the route of V, where V is
    // omitted
    int
    getCheapestInsertSimultRemovalWithTW(Node *U, Node *V, Node *&bestPosition);

    // Preprocess all insertion costs of nodes of route R1 in route R2
    void preprocessInsertions(Route *R1, Route *R2, int nbMoves);

    // Preprocess all insertion costs of nodes of route R1 in route R2
    void preprocessInsertionsWithTW(Route *R1, Route *R2, int nbMoves);

    /* ROUTINES TO UPDATE THE SOLUTIONS */

    // Solution update: Insert U after V
    static void insertNode(Node *U, Node *V);

    // Updates the preprocessed data of a route
    void updateRouteData(Route *myRoute, int nbMoves);

    // Load an initial solution that we will attempt to improve
    void loadIndividual(Individual const &indiv);

    // Export the LS solution back into an individual
    Individual exportIndividual();

    // Performs the actual local search procedure
    void search();

public:
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
