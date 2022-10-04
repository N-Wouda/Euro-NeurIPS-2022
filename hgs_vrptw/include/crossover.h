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
 * Performs a SISR mutation of the passed-in offspring. SISRX first removes
 * strings of clients from each parent based on "Slack Induction String Removals
 * (SISRs)" [1]. Removed clients from one parent are then also removed from the
 * other parent. Both parents are repaired to complete solutions by greedily
 * re-inserting the unplanned clients. <br /> [1]: Christiaens, J., & Vanden
 * Berghe, G. (2020). Slack induction by string removals for vehicle routing
 * problems. Transportation Science, 54(2), 417-433.
 */
Individual stringRemovals(Individual &offspring,
                          Individual const &best,
                          Params const &params,
                          XorShift128 &rng);

Individual brokenPairs(Individual &offspring,
                       Individual const &best,
                       Params const &params,
                       XorShift128 &rng);

#endif  // CROSSOVER_H
