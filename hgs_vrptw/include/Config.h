#ifndef CONFIG_H
#define CONFIG_H

#include <climits>
#include <iosfwd>
#include <string>
#include <utility>

// Stores all the parameters values
struct Config
{
    // TODO make fields const?

    int seed = 0;             // Random seed
    size_t nbIter = 20'000;   // iters without improvement
    int timeLimit = INT_MAX;  // time limit in seconds

    // This was the default until now, but with this value feasible
    // individuals often become infeasible during the local search in
    // educate. With initialTimeWarpPenalty = 10,
    // this does not happen.
    double initialTimeWarpPenalty = 1.0;

    // Set to value > 0, so penalty will get multiplied by this value
    // (instead of default 1.2) if num feasible == 0
    double penaltyBooster = 2.0;

    size_t minimumPopulationSize = 25;
    size_t generationSize = 40;     // max size before culling a generation
    size_t nbElite = 4;             // number of elite individuals in pop
    size_t nbClose = 5;             // # individuals when calculating diversity
    double targetFeasible = 0.2;    // target feasible pop fraction
    size_t repairProbability = 50;  // probability of repair if infeasible

    // The number of iterations without improvements after
    // which the nbGranular is grown
    int growNbGranularAfterNonImprovementIterations = 5'000;

    // The number of iteration after which the nbGranular is grown
    int growNbGranularAfterIterations = 0;

    // The number nbGranular is increased by
    int growNbGranularSize = 0;

    // The number of iterations without improvements after which the
    // minimumPopulationSize is grown
    int growPopulationAfterNonImprovementIterations = 5'000;

    // The number of iteration after which minimumPopulationSize is grown
    int growPopulationAfterIterations = 0;

    // The number minimumPopulationSize is increased by
    int growPopulationSize = 0;

    // Weight for diversity criterion. If 0, weight is set to 1 - nbElite /
    // populationSize
    double diversityWeight = 0.0;

    int nbVeh = INT_MAX;  // Number of vehicles

    // To use dynamic parameters based on instance attributes
    bool useDynamicParameters = false;

    // Granular search parameter, limits the number of moves in the RI local
    // search
    int nbGranular = 40;

    // Probability intensification moves are performed during LS ([0-100])
    int intensificationProbabilityLS = 15;
    bool useSwapStarTW = true;

    // Skip normal swap star based on distance
    bool skipSwapStarDist = false;

    // Margin to take (in degrees 0 - 359 as ints [0 - 65536]) to determine
    // overlap of circle sectors for SWAP*
    int circleSectorOverlapTolerance = 0;

    // Minimum size (in degrees as ints [0 - 65536]) for circle sectors such
    // that even small circle sectors have 'overlap'
    int minCircleSectorSize = static_cast<int>(15 / 360. * 65536);

    // Whether correlation matrix is symmetric
    bool useSymmetricCorrelatedVertices = false;

    explicit Config(size_t nbIter = 20'000,
                    int timeLimit = INT_MAX,
                    int seed = 0,
                    double initialTimeWarpPenalty = 1.0,
                    double penaltyBooster = 2.,
                    size_t minimumPopulationSize = 25,
                    size_t generationSize = 40,
                    size_t nbElite = 4,
                    size_t nbClose = 5,
                    double targetFeasible = 0.2,
                    size_t repairProbability = 50,
                    int growNbGranularAfterNonImprovementIterations = 5'000,
                    int growNbGranularAfterIterations = 0,
                    int growNbGranularSize = 0,
                    int growPopulationAfterNonImprovementIterations = 5'000,
                    int growPopulationAfterIterations = 0,
                    int growPopulationSize = 0,
                    double diversityWeight = 0.,
                    int nbVeh = INT_MAX,
                    bool useDynamicParameters = false,
                    int nbGranular = 40,
                    int intensificationProbabilityLS = 15,
                    bool useSwapStarTW = true,
                    bool skipSwapStarDist = false,
                    int circleSectorOverlapToleranceDegrees = 0,
                    int minCircleSectorSizeDegrees = 15,
                    bool useSymmetricCorrelatedVertices = false)
        : seed(seed),
          nbIter(nbIter),
          timeLimit(timeLimit),
          initialTimeWarpPenalty(initialTimeWarpPenalty),
          penaltyBooster(penaltyBooster),
          minimumPopulationSize(minimumPopulationSize),
          generationSize(generationSize),
          nbElite(nbElite),
          nbClose(nbClose),
          targetFeasible(targetFeasible),
          repairProbability(repairProbability),
          growNbGranularAfterNonImprovementIterations(
              growNbGranularAfterNonImprovementIterations),
          growNbGranularAfterIterations(growNbGranularAfterIterations),
          growNbGranularSize(growNbGranularSize),
          growPopulationAfterNonImprovementIterations(
              growPopulationAfterNonImprovementIterations),
          growPopulationAfterIterations(growPopulationAfterIterations),
          growPopulationSize(growPopulationSize),
          diversityWeight(diversityWeight),
          nbVeh(nbVeh),
          useDynamicParameters(useDynamicParameters),
          nbGranular(nbGranular),
          intensificationProbabilityLS(intensificationProbabilityLS),
          useSwapStarTW(useSwapStarTW),
          skipSwapStarDist(skipSwapStarDist),
          circleSectorOverlapTolerance(circleSectorOverlapToleranceDegrees),
          useSymmetricCorrelatedVertices(useSymmetricCorrelatedVertices)
    {
        auto const overlap = circleSectorOverlapToleranceDegrees / 360. * 65536;
        circleSectorOverlapTolerance = static_cast<int>(overlap);

        auto const minCircleSize = minCircleSectorSizeDegrees / 360. * 65536;
        minCircleSectorSize = static_cast<int>(minCircleSize);
    }
};

#endif  // CONFIG_H
