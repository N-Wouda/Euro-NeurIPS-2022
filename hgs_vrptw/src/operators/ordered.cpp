#include "operators/crossover.h"

#include "Individual.h"
#include "Params.h"
#include "XorShift128.h"

namespace
{
using Parents = std::pair<Individual const *, Individual const *>;

Individual
doOnce(Parents const &parents, Params const &params, XorShift128 &rng)
{
    auto const &tour1 = parents.first->getTour();
    auto const &tour2 = parents.second->getTour();

    std::vector<int> newTour(params.nbClients);
    std::vector<bool> copied(params.nbClients + 1, false);

    // [start, end] marks the clients selected from the first parent. The
    // remaining clients are taken from the second parent.
    size_t start = rng.randint(params.nbClients);
    size_t end = rng.randint(params.nbClients);
    while (end == start)
        end = rng.randint(params.nbClients);

    // Copy in place the elements from start to end (possibly "wrapping around"
    // the end of the array)
    size_t j = start;
    while (j % params.nbClients != (end + 1) % params.nbClients)
    {
        newTour[j % params.nbClients] = tour1[j % params.nbClients];
        copied[newTour[j % params.nbClients]] = true;  // mark as copied
        j++;
    }

    // Fill the remaining elements in the order given by the second parent
    for (int i = 1; i <= params.nbClients; i++)
    {
        int client = tour2[(end + i) % params.nbClients];

        if (!copied[client])  // copy now if not already in tour
        {
            newTour[j % params.nbClients] = client;
            j++;
        }
    }

    return {&params, newTour};
}
}  // namespace

Individual
ordered(Parents const &parents, Params const &params, XorShift128 &rng)
{
    auto const first = doOnce(parents, params, rng);
    auto const second = doOnce(parents, params, rng);

    // Performs another binary tournament with the crossover results. This is
    // not unlike ``Genetic::crossoverOX`` in the baseline.
    return first.cost() < second.cost() ? first : second;
}
