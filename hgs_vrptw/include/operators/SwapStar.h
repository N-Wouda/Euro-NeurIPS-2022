#ifndef SWAPSTAR_H
#define SWAPSTAR_H

#include "LocalSearchOperator.h"
#include "Matrix.h"
#include "Node.h"
#include "Route.h"

#include <array>

/**
 * Explores the SWAP* neighbourhood of [1]. The SWAP* neighbourhood explores
 * free form re-insertions of nodes U and V in the given routes (so the nodes
 * are exchanged between routes, but they are not necessarily inserted in
 * the same place as the other exchanged node). Our implementation of the
 * neighbourhood follows Algorithm 2 of [1] fairly faithfully.
 * <br />
 * Thibaut Vidal. 2022. Hybrid genetic search for the CVRP: Open-source
 * implementation and SWAP* neighborhood. Comput. Oper. Res. 140.
 * https://doi.org/10.1016/j.cor.2021.105643
 */
class SwapStar : public LocalSearchOperator<Route>
{
    struct ThreeBest  // stores three best SWAP* insertion points
    {
        std::array<int, 3> costs = {INT_MAX, INT_MAX, INT_MAX};
        std::array<Node *, 3> locs = {nullptr, nullptr, nullptr};

        void maybeAdd(int costInsert, Node *placeInsert)
        {
            if (costInsert >= costs[2])
                return;

            if (costInsert >= costs[1])
            {
                costs[2] = costInsert;
                locs[2] = placeInsert;
            }
            else if (costInsert >= costs[0])
            {
                costs[2] = costs[1];
                locs[2] = locs[1];
                costs[1] = costInsert;
                locs[1] = placeInsert;
            }
            else
            {
                costs[2] = costs[1];
                locs[2] = locs[1];
                costs[1] = costs[0];
                locs[1] = locs[0];
                costs[0] = costInsert;
                locs[0] = placeInsert;
            }
        }

        void clear()
        {
            costs = {INT_MAX, INT_MAX, INT_MAX};
            locs = {nullptr, nullptr, nullptr};
        }
    };

    struct BestMove  // tracks the best SWAP* move
    {
        int cost = INT_MAX;

        Node *U = nullptr;
        Node *UAfter = nullptr;

        Node *V = nullptr;
        Node *VAfter = nullptr;
    };

    // Preprocesses the given routes. This populates the cache of ThreeBest
    // structs, storing the three best positions in the second route for
    // inserting nodes from the first route.
    void preprocess(Route *R1, Route *R2);

    // Gets the bestPos reinsert point for U in the route of V, assuming V is
    // removed. Returns the cost delta.
    int getBestInsertPoint(Node *U,
                           Node *V,
                           Node *&pos,
                           SwapStar::ThreeBest const &bestPos);

    Matrix<ThreeBest> cache;
    BestMove best;

public:
    void init(Individual const &indiv, Penalties const *penalties) override;

    int test(Route *U, Route *V) override;

    void apply(Route *U, Route *V) override;

    explicit SwapStar(Params const &params)
        : LocalSearchOperator<Route>(params),
          cache(d_params.nbVehicles, d_params.nbClients + 1)
    {
    }
};

#endif  // SWAPSTAR_H
