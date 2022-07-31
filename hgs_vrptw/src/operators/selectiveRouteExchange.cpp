#include "operators/crossover.h"

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

Individual selectiveRouteExchange(
    std::pair<Individual const *, Individual const *> const &parents,
    Params const &params,
    XorShift128 &rng)
{
    return {&params, &rng};  // TODO
}
