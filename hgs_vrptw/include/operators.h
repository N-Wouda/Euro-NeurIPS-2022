#ifndef OPERATORS_H
#define OPERATORS_H

#include "LocalSearch.h"
#include "Node.h"
#include "Params.h"
#include "Route.h"

namespace operators
{
/**
 * Inserts U after V, and updates the solution.
 */
void insertNode(Node *U, Node *V);

/**
 * Swaps U and V, and updates the solution.
 */
void swapNode(Node *U, Node *V);

/**
 * Updates the given route. To be called after swapping nodes/changing the
 * solution.
 */
void updateRouteData(Route *route,
                     int nbMoves,
                     LocalSearch::Penalties const &penalties);
}  // namespace operators

/**
 * Inserts U after V, if that is an improving move.
 */
bool moveSingleClient(int &nbMoves,
                      bool &searchCompleted,
                      Node *nodeU,
                      Node *nodeV,
                      LocalSearch::Penalties const &penalties);

/**
 * Inserts (U -> X) after V, if that is an improving move.
 */
bool moveTwoClients(int &nbMoves,
                    bool &searchCompleted,
                    Node *nodeU,
                    Node *nodeV,
                    LocalSearch::Penalties const &penalties);

/**
 * Inserts U -> X after V (as V -> X -> U), if that is an improving move.
 */
bool moveTwoClientsReversed(int &nbMoves,
                            bool &searchCompleted,
                            Node *nodeU,
                            Node *nodeV,
                            LocalSearch::Penalties const &penalties);

/**
 * Swaps (U -> X) and (V -> Y), if that is an improving move.
 */
bool swapTwoClientPairs(int &nbMoves,
                        bool &searchCompleted,
                        Node *nodeU,
                        Node *nodeV,
                        LocalSearch::Penalties const &penalties);

/**
 * Swaps (U -> X) and V, if that is an improving move.
 */
bool swapTwoClientsForOne(int &nbMoves,
                          bool &searchCompleted,
                          Node *nodeU,
                          Node *nodeV,
                          LocalSearch::Penalties const &penalties);

/**
 * Swaps U and V, if that is an improving move.
 */
bool swapTwoSingleClients(int &nbMoves,
                          bool &searchCompleted,
                          Node *nodeU,
                          Node *nodeV,
                          LocalSearch::Penalties const &penalties);

/**
 * Replaces U -> X and V -> Y by U -> Y and V -> X, if that is an improving
 * move. Assumes U and V do not belong to the same route (if they do, this is a
 * no-op).
 */
bool twoOptBetweenTrips(int &nbMoves,
                        bool &searchCompleted,
                        Node *nodeU,
                        Node *nodeV,
                        LocalSearch::Penalties const &penalties);

/**
 * Replaces U -> X and V -> Y by U -> V and X -> Y, if that is an improving
 * move. Assumes U and V belong to the same route (if not, this is a no-op).
 */
bool twoOptWithinTrip(int &nbMoves,
                      bool &searchCompleted,
                      Node *nodeU,
                      Node *nodeV,
                      LocalSearch::Penalties const &penalties);

/**
 * Performs the best moveSingleClient move between routes U and V.
 */
bool relocateStar(int &nbMoves,
                  bool &searchCompleted,
                  Route *routeU,
                  Route *routeV,
                  LocalSearch::Penalties const &penalties);

#endif  // OPERATORS_H
