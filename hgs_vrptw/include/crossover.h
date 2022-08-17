#ifndef CROSSOVER_H
#define CROSSOVER_H

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <unordered_set>
#include <vector>

namespace crossover
{
/**
 * Greedily inserts the unplanned clients into non-empty routes.
 */
void greedyRepair(std::vector<std::vector<int>> &routes,
                  std::unordered_set<int> const &unplanned,
                  Params const &params);
}  // namespace crossover

/**
 * Does two ordered crossovers of the given parents (binary tournament). Each
 * crossover randomly selects a subset of clients from the first parent, and the
 * rest comes from the second parent.
 */
Individual orderedExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng);

/**
 * Performs two SREX [1] crossovers of the given parents (binary tournament).
 * This was one of ORTEC's DIMACS contributions.
 * <br />
 * [1]: Yuichi Nagata and Shigenobu Kobayashi. "A memetic algorithm for the
 * pickup and delivery problem with time windows using selective route exchange
 * crossover". In: International Conference on Parallel Problem Solving from
 * Nature. Springer. 2010, pp. 536–545.
 */
Individual selectiveRouteExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng);

/**
 * Creates an offspring by selecting alternately the next client of the first
 * parent and the next client of the second parent, omitting clients already
 * present in the offspring.
 * <br />
 * P. Larrañaga , C. Kuijpers, R. Murga, Y Yurramendi. 1996. "Learning Bayesian
 * network structures by searching for the best ordering with genetic
 * algorithms". IEEE Transactions on Systems, Man and Cybernetics. 26, pp.
 * 487-493.
 */
Individual alternatingExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng);

#endif  // CROSSOVER_H
