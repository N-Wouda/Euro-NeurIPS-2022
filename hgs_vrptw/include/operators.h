#ifndef OPERATORS_H
#define OPERATORS_H

#include "LocalSearch.h"
#include "Node.h"
#include "Params.h"
#include "Route.h"

namespace operators  // TODO obsolete
{
/**
 * Inserts U after V, and updates the solution.
 */
void insertNode(Node *U, Node *V);

/**
 * Swaps U and V, and updates the solution.
 */
void swapNode(Node *U, Node *V);
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

#endif  // OPERATORS_H
