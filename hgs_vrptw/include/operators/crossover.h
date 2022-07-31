#ifndef CROSSOVER_H
#define CROSSOVER_H

#include "Individual.h"

/**
 * TODO OX
 */
Individual
ordered(std::pair<Individual const *, Individual const *> const &parents,
        Params const &params,
        XorShift128 &rng);

/**
 * TODO SREX
 */
Individual selectiveRouteExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng);

#endif  // CROSSOVER_H
