#include "Result.h"
#include "Individual.h"

#include <fstream>
#include <vector>

void Result::exportBestKnownSolution(std::string const &path) const
{
    // Compare to current best known solution, if exists.
    double currCost;
    std::vector<std::vector<int>> currSol;
    bool readOK = Individual::readCVRPLibFormat(path, currSol, currCost);

    auto const *bestSol = getBestFound();

    if (!readOK || bestSol->myCostSol.penalizedCost < currCost)
        bestSol->exportCVRPLibFormat(path);
}

Individual const *Result::getBestFound() const
{
    double cost = UINT_MAX;
    Individual const *best = nullptr;

    for (auto const *indiv : feasible)
        if (indiv->myCostSol.penalizedCost < cost)
        {
            best = indiv;
            cost = indiv->myCostSol.penalizedCost;
        }

    return best;
}
