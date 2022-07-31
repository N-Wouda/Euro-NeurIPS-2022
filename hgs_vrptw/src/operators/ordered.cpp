#include "operators/crossover.h"

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

#include <ranges>

namespace
{
using Parents = std::pair<Individual const *, Individual const *>;
using offsets = std::pair<unsigned, unsigned>;

offsets getStartEnd(int n, XorShift128 &rng)
{
    size_t start = rng.randint(n);
    size_t end = rng.randint(n);
    while (end == start)
        end = rng.randint(n);

    return std::make_pair(start, end);
}

Individual
doOnce(Parents const &parents, Params const &params, offsets const &slice)
{
    auto const &tour1 = parents.first->getTour();
    auto const &tour2 = parents.second->getTour();

    auto const [start, end] = slice;

    std::vector<int> crossoverTour(params.nbClients);
    std::vector<bool> copied(params.nbClients + 1, false);

    // Copy clients from start to end (possibly "wrapping around" the end)
    size_t j = start;
    for (; j % params.nbClients != (end + 1) % params.nbClients; ++j)
    {
        crossoverTour[j % params.nbClients] = tour1[j % params.nbClients];
        copied[crossoverTour[j % params.nbClients]] = true;  // mark as copied
    }

    // Fill the remaining clients in the order given by the second parent
    for (int idx : std::views::iota(1, params.nbClients + 1))
    {
        int const client = tour2[(end + idx) % params.nbClients];
        if (!copied[client])
            crossoverTour[j++ % params.nbClients] = client;
    }

    return {&params, crossoverTour};
}
}  // namespace

Individual
ordered(Parents const &parents, Params const &params, XorShift128 &rng)
{
    // Performs another binary tournament with the crossover results. This is
    // not unlike ``Genetic::crossoverOX`` in the baseline.
    auto const slice1 = getStartEnd(params.nbClients, rng);
    auto const indiv1 = doOnce(parents, params, slice1);

    auto const slice2 = getStartEnd(params.nbClients, rng);
    auto const indiv2 = doOnce(parents, params, slice2);

    return indiv1.cost() < indiv2.cost() ? indiv1 : indiv2;
}
