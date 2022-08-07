#ifndef OPERATORS_H
#define OPERATORS_H

#include "Node.h"
#include "Penalties.h"
#include "Route.h"

namespace operators
{
/**
 * Cost delta of inserting U after V.
 */
int singleMoveCost(Node *nodeU, Node *nodeV, Penalties const &penalties);
}  // namespace operators

/**
 * Inserts U after V, if that is an improving move.
 */
bool moveSingleClient(Node *nodeU, Node *nodeV, Penalties const &penalties);

/**
 * Inserts (U -> X) after V, if that is an improving move.
 */
bool moveTwoClients(Node *nodeU, Node *nodeV, Penalties const &penalties);

/**
 * Inserts U -> X after V (as V -> X -> U), if that is an improving move.
 */
bool moveTwoClientsReversed(Node *nodeU,
                            Node *nodeV,
                            Penalties const &penalties);

/**
 * Swaps (U -> X) and (V -> Y), if that is an improving move.
 */
bool swapTwoClientPairs(Node *nodeU, Node *nodeV, Penalties const &penalties);

/**
 * Swaps (U -> X) and V, if that is an improving move.
 */
bool swapTwoClientsForOne(Node *nodeU, Node *nodeV, Penalties const &penalties);

/**
 * Swaps U and V, if that is an improving move.
 */
bool swapTwoSingleClients(Node *nodeU, Node *nodeV, Penalties const &penalties);

/**
 * Replaces U -> X and V -> Y by U -> Y and V -> X, if that is an improving
 * move. Assumes U and V do not belong to the same route (if they do, this is a
 * no-op).
 */
bool twoOptBetweenTrips(Node *nodeU, Node *nodeV, Penalties const &penalties);

/**
 * Replaces U -> X and V -> Y by U -> V and X -> Y, if that is an improving
 * move. Assumes U and V belong to the same route (if not, this is a no-op).
 */
bool twoOptWithinTrip(Node *nodeU, Node *nodeV, Penalties const &penalties);

/**
 * Performs the best moveSingleClient move between routes U and V.
 */
bool relocateStar(Route *routeU, Route *routeV, Penalties const &penalties);

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
bool swapStar(Route *routeU, Route *routeV, Penalties const &penalties);

#endif  // OPERATORS_H
