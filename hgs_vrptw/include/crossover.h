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
 * Performs a SISRX crossover of the given parents (binary tournament). SISRX
 * first removes strings of clients from each parent based on "Slack Induction
 * String Removals (SISRs)" [1]. Removed clients from one parent are then
 * also removed from the other parent. Both parents are repaired to complete
 * solutions by greedily re-inserting the unplanned clients, while randomly
 * skipping insertion positions.
 * <br />
 * [1]: Christiaens, J., & Vanden Berghe, G. (2020). Slack induction by string
 * removals for vehicle routing problems. Transportation Science, 54(2),
 * 417-433.
 */
Individual stringRemovalExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng);

namespace crossover
{
/**
 * Greedily inserts the unplanned clients into the routes, while skipping
 * positions with probability blinkRate (see stringRemovalExchange).
 */
void greedyRepairWithBlinks(std::vector<std::vector<int>> &routes,
                            std::vector<int> const &unplanned,
                            size_t blinkRate,
                            Params const &params,
                            XorShift128 &rng);
}  // namespace crossover
#endif  // CROSSOVER_H
