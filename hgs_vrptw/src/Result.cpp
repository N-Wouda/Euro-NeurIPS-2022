#include "Result.h"
#include "Individual.h"

void Result::writeBestKnowSolution(std::string const &path) const
{
    // Compare to current best known solution, if exists.
    double currCost;

    try
    {
        auto [_, cost] = Individual::readCVRPLibFormat(path);
        currCost = cost;
    }
    catch (std::runtime_error const &e)
    {
        currCost = UINT_MAX;
    }

    auto const *bestSol = getBestFound();

    if (bestSol->costs.penalizedCost < currCost)
        bestSol->exportCVRPLibFormat(path);
}

Individual const *Result::getBestFound() const
{
    return feasible[0];  // since they are sorted by cost.
}
