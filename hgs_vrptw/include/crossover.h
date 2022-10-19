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

/**
 * Performs edge assembly exchange crossover operator. We take the edges from
 * the parent in XOR fashion, and construct a cycle that alternates edges from
 * parent A and B, with B in the opposite direction from A. This cycle is
 * reinserted into A and the resulting routes are repaired. May fail to find a
 * solution and returns parent A in this case. Largely based on: Yuichi Nagata,
 * Olli Bräysy, Wout Dullaert, "A penalty-based edge assembly memetic algorithm
 * for the vehicle routing problem with time windows". In: Computers &
 * Operations research,Volume 37, Issue 4,Pages 724-737,
 * https://doi.org/10.1016/j.cor.2009.06.022.
 */
Individual
edgeAssembly(std::pair<Individual const *, Individual const *> const &parents,
             Params const &params,
             XorShift128 &rng);

#endif  // CROSSOVER_H
