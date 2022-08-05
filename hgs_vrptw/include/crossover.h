#ifndef CROSSOVER_H
#define CROSSOVER_H

#include "Individual.h"
#include <unordered_set>

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

/**
 * Performs two SISRX crossovers of the given parents (binary tournament). SISRX
 * first removes strings of clients from each parent based on "Slack Induction
 * String Removals (SISR)" [2]. Removed clients from one parent are then
 * also removed from the other parent. Both parents are repaired to complete
 * solutions by greedily re-inserting the unassinged clients.
 * <br />
 * [2]: Christiaens, J., & Vanden Berghe, G. (2020). Slack induction by string
 * removals for vehicle routing problems. Transportation Science, 54(2),
 * 417-433.
 */
Individual stringRemovalExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng);

struct InsertPos  // best insert position, used to plan unplanned clients
{
    int deltaCost;
    std::vector<int> *route;
    size_t offset;
};

/**
 * Evaluates the cost change of inserting client between prev and next.
 */
int deltaCost(int client, int prev, int next, Params const &params);

/**
 * Finds the previous and next client index given the current route idx.
 */
std::pair<size_t, size_t> findPrevNext(std::vector<int> const &route,
                                       size_t idx);
#endif  // CROSSOVER_H
