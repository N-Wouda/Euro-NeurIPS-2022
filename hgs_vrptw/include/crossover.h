#ifndef CROSSOVER_H
#define CROSSOVER_H

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <vector>

namespace crossover
{
/**
 * Greedily inserts the unplanned clients into non-empty routes.
 */
void greedyRepair(std::vector<std::vector<int>> &routes,
                  std::vector<int> const &unplanned,
                  Params const &params);
}  // namespace crossover

/**
 * Performs two SREX crossovers of the given parents (binary tournament). This
 * was one of ORTEC's DIMACS contributions.
 * <br />
 * Yuichi Nagata and Shigenobu Kobayashi. "A memetic algorithm for the pickup
 * and delivery problem with time windows using selective route exchange
 * crossover". In: International Conference on Parallel Problem Solving from
 * Nature. Springer. 2010, pp. 536–545.
 */
Individual selectiveRouteExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng);

/**
 * Performs one broken pair crossover of the given parents. A client is removed
 * from the worst parents if its successor is not identical to the client's
 * successor in the other parent. Removed clients are then greedily re-inserted.
 */
Individual brokenPairsExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng);

Individual
edgeAssembly(std::pair<Individual const *, Individual const *> const &parents,
             Params const &params,
             XorShift128 &rng,
             bool strategy);

#endif  // CROSSOVER_H
