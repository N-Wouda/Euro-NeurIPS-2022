#ifndef CROSSOVER_H
#define CROSSOVER_H

#include "Individual.h"

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
 * Performs two SREX [1] crossovers the given parents (binary tournament). This
 * was one of ORTEC's DIMACS contributions.
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

#endif  // CROSSOVER_H
