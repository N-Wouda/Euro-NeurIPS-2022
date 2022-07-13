#include "Result.h"
#include "Individual.h"

#include <fstream>
#include <vector>

void Result::writeBestKnowSolution(std::string const &path) const
{
    // Compare to current best known solution, if exists.
    double currCost;
    std::vector<std::vector<int>> currSol;
    bool readOK = Individual::readCVRPLibFormat(path, currSol, currCost);

    auto const *bestSol = getBestFound();

    if (!readOK || bestSol->costs.penalizedCost < currCost)
        bestSol->exportCVRPLibFormat(path);
}

Individual const *Result::getBestFound() const
{
    return feasible[0];  // since they are sorted by cost.
}
