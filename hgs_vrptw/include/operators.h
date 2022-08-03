#ifndef OPERATORS_H
#define OPERATORS_H

#include "LocalSearch.h"
#include "Params.h"

namespace operators
{
/**
 * Inserts U after V, and updates the solution.
 */
void insertNode(LocalSearch::Node *U, LocalSearch::Node *V);

/**
 * Swaps U and V, and updates the solution.
 */
void swapNode(LocalSearch::Node *U, LocalSearch::Node *V);

/**
 * Updates the given route. To be called after swapping nodes/changing the
 * solution.
 */
void updateRouteData(LocalSearch::Route *myRoute,
                     int nbMoves,
                     LocalSearch::Penalties const &penalties,
                     Params const &params);
}  // namespace operators

/**
 * Inserts U after V, if that is an improving move.
 */
bool moveSingleClient(int &nbMoves,
                      bool &searchCompleted,
                      LocalSearch::Node *nodeU,
                      LocalSearch::Node *nodeV,
                      LocalSearch::Penalties const &penalties,
                      Params const &params);

/**
 * Inserts (U -> X) after V, if that is an improving move.
 */
bool moveTwoClients(int &nbMoves,
                    bool &searchCompleted,
                    LocalSearch::Node *nodeU,
                    LocalSearch::Node *nodeV,
                    LocalSearch::Penalties const &penalties,
                    Params const &params);

/**
 * Inserts U -> X after V (as V -> X -> U), if that is an improving move.
 */
bool moveTwoClientsReversed(int &nbMoves,
                            bool &searchCompleted,
                            LocalSearch::Node *nodeU,
                            LocalSearch::Node *nodeV,
                            LocalSearch::Penalties const &penalties,
                            Params const &params);

/**
 * Swaps (U -> X) and (V -> Y), if that is an improving move.
 */
bool swapTwoClientPairs(int &nbMoves,
                        bool &searchCompleted,
                        LocalSearch::Node *nodeU,
                        LocalSearch::Node *nodeV,
                        LocalSearch::Penalties const &penalties,
                        Params const &params);

/**
 * Swaps (U -> X) and V, if that is an improving move.
 */
bool swapTwoClientsForOne(int &nbMoves,
                          bool &searchCompleted,
                          LocalSearch::Node *nodeU,
                          LocalSearch::Node *nodeV,
                          LocalSearch::Penalties const &penalties,
                          Params const &params);

/**
 * Swaps U and V, if that is an improving move.
 */
bool swapTwoSingleClients(int &nbMoves,
                          bool &searchCompleted,
                          LocalSearch::Node *nodeU,
                          LocalSearch::Node *nodeV,
                          LocalSearch::Penalties const &penalties,
                          Params const &params);

#endif  // OPERATORS_H
