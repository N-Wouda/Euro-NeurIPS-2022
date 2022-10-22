#ifndef CONFIG_H
#define CONFIG_H

#include <climits>
#include <iosfwd>
#include <string>
#include <utility>

// Stores all the parameters values
struct Config
{
    int seed = 0;                    // Random seed
    size_t nbIter = 10'000;          // iters without improvement
    int timeLimit = INT_MAX;         // time limit in seconds
    bool collectStatistics = false;  // collect runtime statistics?

    // This was the default until now, but with this value feasible individuals
    // often become infeasible during the local search in educate. This does not
    // happen with initialTimeWarpPenalty = 10.
    size_t initialTimeWarpPenalty = 1;

    size_t nbPenaltyManagement = 100;  // manage penalties every # iterations
    double feasBooster = 2.0;          // special increase penalty if no feas
    double penaltyIncrease = 1.2;      // regular increase if below target feas
    double penaltyDecrease = 0.85;     // regular decrease if above target feas

    size_t minPopSize = 25;
    size_t generationSize = 40;   // max size before culling a generation
    size_t nbElite = 4;           // number of elite individuals in pop
    double lbDiversity = 0.1;     // minimum pct difference in parent diversity
    double ubDiversity = 0.5;     // maximum pct difference in parent diversity
    size_t nbClose = 5;           // # individuals when calculating diversity
    double targetFeasible = 0.4;  // target feasible pop fraction
    size_t nbKeepOnRestart = 0;   // # individuals to keep when restarting

    size_t repairProbability = 50;  // probability of repair if infeasible
    size_t repairBooster = 10;      // penalty booster when repairing

    size_t selectProbability = 90;  // offspring selection probability

    int nbVeh = INT_MAX;  // Number of vehicles

    // Granular search parameter, limits the number of moves in the RI local
    // search
    size_t nbGranular = 40;

    // See Vidal 2012, HGS for VRPTW. Multiplied by 10 for integer arithmetic.
    int weightWaitTime = 2;   // weight for wait-time nearness
    int weightTimeWarp = 10;  // weight for time warp nearness

    bool shouldIntensify = true;  // try to further improve a new best solution?

    // Margin to take (in degrees 0 - 359 as ints [0 - 65536]) to determine
    // overlap of circle sectors for SWAP*
    int circleSectorOverlapTolerance = 0;

    // Minimum size (in degrees as ints [0 - 65536]) for circle sectors such
    // that even small circle sectors have 'overlap'
    int minCircleSectorSize = static_cast<int>(15 / 360. * 65536);

    // Number of nodes we improve by enumeration in LS postprocessing
    size_t postProcessPathLength = 6;

    explicit Config(int seed = 0,
                    size_t nbIter = 10'000,
                    int timeLimit = INT_MAX,
                    bool collectStatistics = false,
                    size_t initialTimeWarpPenalty = 1,
                    size_t nbPenaltyManagement = 100,
                    double feasBooster = 2.,
                    double penaltyIncrease = 1.2,
                    double penaltyDecrease = 0.85,
                    size_t minPopSize = 25,
                    size_t generationSize = 40,
                    size_t nbElite = 4,
                    double lbDiversity = 0.1,
                    double ubDiversity = 0.5,
                    size_t nbClose = 5,
                    double targetFeasible = 0.4,
                    size_t nbKeepOnRestart = 0,
                    size_t repairProbability = 50,
                    size_t repairBooster = 10,
                    size_t selectProbability = 90,
                    int nbVeh = INT_MAX,
                    size_t nbGranular = 40,
                    int weightWaitTime = 2,
                    int weightTimeWarp = 10,
                    bool shouldIntensify = true,
                    int circleSectorOverlapToleranceDegrees = 0,
                    int minCircleSectorSizeDegrees = 15,
                    size_t postProcessPathLength = 6)
        : seed(seed),
          nbIter(nbIter),
          timeLimit(timeLimit),
          collectStatistics(collectStatistics),
          initialTimeWarpPenalty(initialTimeWarpPenalty),
          nbPenaltyManagement(nbPenaltyManagement),
          feasBooster(feasBooster),
          penaltyIncrease(penaltyIncrease),
          penaltyDecrease(penaltyDecrease),
          minPopSize(minPopSize),
          generationSize(generationSize),
          nbElite(nbElite),
          lbDiversity(lbDiversity),
          ubDiversity(ubDiversity),
          nbClose(nbClose),
          targetFeasible(targetFeasible),
          nbKeepOnRestart(nbKeepOnRestart),
          repairProbability(repairProbability),
          repairBooster(repairBooster),
          selectProbability(selectProbability),
          nbVeh(nbVeh),
          nbGranular(nbGranular),
          weightWaitTime(weightWaitTime),
          weightTimeWarp(weightTimeWarp),
          shouldIntensify(shouldIntensify),
          postProcessPathLength(postProcessPathLength)
    {
        auto const overlap = circleSectorOverlapToleranceDegrees / 360. * 65536;
        circleSectorOverlapTolerance = static_cast<int>(overlap);

        auto const minCircleSize = minCircleSectorSizeDegrees / 360. * 65536;
        minCircleSectorSize = static_cast<int>(minCircleSize);
    }
};

#endif  // CONFIG_H
