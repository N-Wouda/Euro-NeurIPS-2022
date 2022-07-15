/*MIT License

Original HGS-CVRP code: Copyright(c) 2020 Thibaut Vidal
Additional contributions: Copyright(c) 2022 ORTEC

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef PARAMS_H
#define PARAMS_H

#include "Matrix.h"
#include "XorShift128.h"

#include <chrono>
#include <climits>
#include <ctime>
#include <string>
#include <vector>

#define MY_EPSILON 0.00001  // Used to avoid numerical instabilities
#define PI 3.14159265359    // Pi, with 11 decimal precision

// Structure of a Client, including its index, position, and all other variables
// and parameters
struct Client
{
    int custNum;          // Index of the client
    int coordX;           // Coordinate X
    int coordY;           // Coordinate Y
    int serviceDuration;  // Service duration
    int demand;           // Demand
    int earliestArrival;  // Earliest arrival (when using time windows)
    int latestArrival;    // Latest arrival (when using time windows)

    // Release time (when using time windows, route containing this customer
    // cannot depart before this time)
    int releaseTime;

    // Polar angle of the client around the depot (starting at east, moving
    // counter-clockwise), measured in degrees and truncated for convenience
    int polarAngle;
};

// Class that stores all the parameters (from the command line) (in Config) and
// data of the instance needed by the algorithm
class Params
{
private:
    void setup();

public:
    // Stores all the parameters values (given by using the command line)
    struct Config
    {
        // Number of iterations without improvement until termination.
        int nbIter = 20'000;

        // CPU time limit until termination in seconds.
        int timeLimit = INT_MAX;

        // If True, measure wall clock time rather than CPU time
        bool useWallClockTime = false;

        // Parameters for the Construction Heuristics

        // Proportion of individuals constructed by nearest-first
        double fractionGeneratedNearest = 0.05;

        // Proportion of individuals constructed by furthest-first
        double fractionGeneratedFurthest = 0.05;

        // Proportion of individuals constructed by sweep
        double fractionGeneratedSweep = 0.05;

        // Proportion of individuals constructed randomly
        double fractionGeneratedRandomly = 0.85;

        // Fill rate in BKS is always more than 40%, so I don't think less than
        // this would make sense. The maximum vehicle usage is 40% (100/250
        // routes, see SINTEF BKS), so take 60% to have some margin (otherwise
        // all remaining orders will be in last route)
        int minSweepFillPercentage = 60;

        // In the instance I checked vehicle capacity was 1000, so max 5% could
        // make sense.
        int maxToleratedCapacityViolation = 50;

        // No real feeling yet for what value makes sense.
        int maxToleratedTimeWarp = 100;

        // This was the default until now, but with this value feasible
        // individuals often become infeasible during the local search in
        // doLocalSearchAndAddIndividual. With initialTimeWarpPenalty = 10, this
        // does not happen.
        double initialTimeWarpPenalty = 1.0;

        // Set to value > 0, so penalty will get multiplied by this value
        // (instead of default 1.2) if num feasible == 0
        double penaltyBooster = 2.;

        // Parameters of the Genetic Algorithm

        size_t minimumPopulationSize = 25;  // Minimum population size

        // Number of solutions created before reaching the maximum population
        // size (i.e., generation size)
        size_t generationSize = 40;

        // Number of elite individuals (reduced in HGS-2020)
        size_t nbElite = 4;

        // Number of closest solutions/individuals considered when calculating
        // diversity contribution
        size_t nbClose = 5;

        // Reference proportion for the number of feasible individuals, used
        // for the adaptation of the penalty parameters
        double targetFeasible = 0.2;

        // Integer (0-100) representing repair probability if individual is
        // infeasible after local search
        int repairProbability = 50;

        // The number of iterations without improvements after
        // which the nbGranular is grown
        int growNbGranularAfterNonImprovementIterations = 5000;

        // The number of iteration after which the nbGranular is grown
        int growNbGranularAfterIterations = 0;

        // The number nbGranular is increased by
        int growNbGranularSize = 0;

        // The number of iterations without improvements after which the
        // minimumPopulationSize is grown
        int growPopulationAfterNonImprovementIterations = 5000;

        // The number of iteration after which minimumPopulationSize is grown
        int growPopulationAfterIterations = 0;

        // The number minimumPopulationSize is increased by
        int growPopulationSize = 0;

        // Weight for diversity criterion. If 0, weight is set to 1 - nbElite /
        // populationSize
        double diversityWeight = 0.;

        // Other parameters

        int nbVeh = INT_MAX;  // Number of vehicles

        // To use dynamic parameters based on instance attributes
        bool useDynamicParameters = false;
        std::string pathSolution;  // Solution file path

        // Granular search parameter, limits the number of moves in the RI local
        // search
        int nbGranular = 40;

        // Probability intensification moves are performed during LS ([0-100])
        int intensificationProbabilityLS = 15;
        bool useSwapStarTW = true;  // Use TW swap star

        // Skip normal swap star based on distance
        bool skipSwapStarDist = false;

        // Margin to take (in degrees 0 - 359) to determine overlap of circle
        // sectors for SWAP*
        int circleSectorOverlapToleranceDegrees = 0;

        // Minimum size (in degrees) for circle sectors such that even small
        // circle sectors have 'overlap'
        int minCircleSectorSizeDegrees = 15;
        int seed = 0;              // Random seed. Default value: 0
        std::string pathInstance;  // Instance file path

        // Whether correlation matrix is symmetric
        bool useSymmetricCorrelatedVertices = false;

        // Whether to repeat the algorithm when max nr of iter is reached, but
        // time limit is not
        bool doRepeatUntilTimeLimit = true;
    };

    Config config;    // Stores all the parameter values
    XorShift128 rng;  // Fast random number generator

    // Start wall clock time of this object (should be constructed at start of
    // program)
    std::chrono::system_clock::time_point startWallClockTime;
    std::clock_t startCPUTime;  // Start CPU time of this object

    // Penalty for one unit of capacity excess (adapted through the search)
    double penaltyCapacity;

    // Penalty for one unit waiting time (adapted through the search)
    double penaltyWaitTime;

    // Penalty for one unit time warp (adapted through the search)
    double penaltyTimeWarp;

    // Weight for waiting time in defining the neighbourhood proximities
    double proximityWeightWaitTime;

    // Weight for time warp in defining the neighbourhood proximities
    double proximityWeightTimeWarp;

    std::string instanceName;
    bool isExplicitDistanceMatrix;  // Has explicit distances (non-euclidean)
    int nbClients;                  // Number of clients (excluding the depot)
    int nbVehicles;                 // Number of vehicles
    int vehicleCapacity;            // Capacity limit
    int totalDemand;                // Total demand required by the clients
    int maxDemand;                  // Maximum demand of a client
    int maxDist;                    // Maximum distance between two clients
    std::vector<Client> cli;        // Client (+depot) information
    Matrix timeCost;                // Distance matrix (+depot)

    // For each client, other clients sorted by proximity (size nbClients + 1,
    // but nothing stored for the depot!)
    std::vector<std::vector<std::pair<double, int>>> orderProximities;

    // Neighborhood restrictions: For each client, list of nearby clients (size
    // nbClients + 1, but nothing stored for the depot!)
    std::vector<std::vector<int>> correlatedVertices;

    // Tolerance when determining circle sector overlap (0 - 65536)
    int circleSectorOverlapTolerance;

    // Minimum circle sector size to enforce (for nonempty routes) (0 - 65536)
    int minCircleSectorSize;

    explicit Params(Config &config);

    Params(std::string const &instancePath,
           std::string const &solPath,
           int nbIter = 20'000,
           int timeLimit = INT_MAX,
           int seed = 0,
           bool useWallClockTime = false,
           double fractionGeneratedNearest = 0.05,
           double fractionGeneratedFurthest = 0.05,
           double fractionGeneratedSweep = 0.05,
           double fractionGeneratedRandomly = 0.85,
           int minSweepFillPercentage = 60,
           int maxToleratedCapacityViolation = 50,
           int maxToleratedTimeWarp = 100,
           double initialTimeWarpPenalty = 1.0,
           double penaltyBooster = 2.,
           size_t minimumPopulationSize = 25,
           size_t generationSize = 40,
           size_t nbElite = 4,
           size_t nbClose = 5,
           double targetFeasible = 0.2,
           int repairProbability = 50,
           int growNbGranularAfterNonImprovementIterations = 5000,
           int growNbGranularAfterIterations = 0,
           int growNbGranularSize = 0,
           int growPopulationAfterNonImprovementIterations = 5000,
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
           bool useSymmetricCorrelatedVertices = false,
           bool doRepeatUntilTimeLimit = true);

    // Get time elapsed since start of program
    [[nodiscard]] double getElapsedTime() const;

    // Whether time limit is exceeded
    [[nodiscard]] bool isTimeLimitExceeded() const;

    // Calculate, for all vertices, the correlation for the nbGranular closest
    // vertices
    void SetCorrelatedVertices();
};

#endif
