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

#ifndef COMMANDLINE_H
#define COMMANDLINE_H

#include "Params.h"

#include <iostream>  // needed for parse() below
#include <string>

// Class interacting with the command line
// Parameters can be read from the command line and information can be written
// to the command line
class CommandLine
{
    int argc;
    char **argv;

public:
    // argc is the number of command line arguments
    // argv are the command line arguments:
    //		1) The path to the genvrp executable to run
    //		2) The path to the instance to consider
    //		3) The path to the file where the solution will be stored
    //      4) Possibly combinations of command line argument descriptions with
    //         their value (counted as 2 per argument in argc)
    CommandLine(int argc, char **argv) : argc(argc), argv(argv)
    {
        // Check if the number of arguments is odd and at least three, since the
        // three paths should at least be given
        if (argc % 2 != 1 || argc < 3)
        {
            displayHelp();

            throw std::string("Incorrect number of arguments");
        }
    }

    // Extracts run configurations from command line arguments
    Params parse()
    {
        Params::Config config;

        // Get the paths of the instance and the solution
        config.pathInstance = std::string(argv[1]);
        config.pathSolution = std::string(argv[2]);

        // Go over all possible command line arguments and store their
        // values Explanations per command line argument can be found at
        // their variable declaration, as well as in displayHelp()
        for (int i = 3; i < argc; i += 2)
        {
            if (std::string(argv[i]) == "-t")
                config.timeLimit = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-useWallClockTime")
                config.useWallClockTime = atoi(argv[i + 1]) != 0;
            else if (std::string(argv[i]) == "-it")
                config.nbIter = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-seed")
                config.seed = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-veh")
                config.nbVeh = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-useDynamicParameters")
                config.useDynamicParameters = atoi(argv[i + 1]) != 0;
            else if (std::string(argv[i]) == "-nbGranular")
                config.nbGranular = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-fractionGeneratedNearest")
                config.fractionGeneratedNearest = atof(argv[i + 1]);
            else if (std::string(argv[i]) == "-fractionGeneratedFurthest")
                config.fractionGeneratedFurthest = atof(argv[i + 1]);
            else if (std::string(argv[i]) == "-fractionGeneratedSweep")
                config.fractionGeneratedSweep = atof(argv[i + 1]);
            else if (std::string(argv[i]) == "-fractionGeneratedRandomly")
                config.fractionGeneratedRandomly = atof(argv[i + 1]);
            else if (std::string(argv[i]) == "-minSweepFillPercentage")
                config.minSweepFillPercentage = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-maxToleratedCapacityViolation")
                config.maxToleratedCapacityViolation = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-maxToleratedTimeWarp")
                config.maxToleratedTimeWarp = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-initialTimeWarpPenalty")
                config.initialTimeWarpPenalty = atof(argv[i + 1]);
            else if (std::string(argv[i]) == "-penaltyBooster")
                config.penaltyBooster = atof(argv[i + 1]);
            else if (std::string(argv[i]) == "-useSymmetricCorrelatedVertices")
                config.useSymmetricCorrelatedVertices = atoi(argv[i + 1]) != 0;
            else if (std::string(argv[i]) == "-doRepeatUntilTimeLimit")
                config.doRepeatUntilTimeLimit = atoi(argv[i + 1]) != 0;
            else if (std::string(argv[i]) == "-minimumPopulationSize")
                config.minimumPopulationSize
                    = static_cast<size_t>(atoi(argv[i + 1]));
            else if (std::string(argv[i]) == "-generationSize")
                config.generationSize = static_cast<size_t>(atoi(argv[i + 1]));
            else if (std::string(argv[i]) == "-nbElite")
                config.nbElite = static_cast<size_t>(atoi(argv[i + 1]));
            else if (std::string(argv[i]) == "-nbClose")
                config.nbClose = static_cast<size_t>(atoi(argv[i + 1]));
            else if (std::string(argv[i]) == "-targetFeasible")
                config.targetFeasible = atof(argv[i + 1]);
            else if (std::string(argv[i]) == "-repairProbability")
                config.repairProbability = atoi(argv[i + 1]);
            else if (std::string(argv[i])
                     == "-growNbGranularAfterNonImprovementIterations")
                config.growNbGranularAfterNonImprovementIterations
                    = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-growNbGranularAfterIterations")
                config.growNbGranularAfterIterations = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-growNbGranularSize")
                config.growNbGranularSize = atoi(argv[i + 1]);
            else if (std::string(argv[i])
                     == "-growPopulationAfterNonImprovementIterations")
                config.growPopulationAfterNonImprovementIterations
                    = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-growPopulationAfterIterations")
                config.growPopulationAfterIterations = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-growPopulationSize")
                config.growPopulationSize = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-intensificationProbabilityLS")
                config.intensificationProbabilityLS = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-diversityWeight")
                config.diversityWeight = atof(argv[i + 1]);
            else if (std::string(argv[i]) == "-useSwapStarTW")
                config.useSwapStarTW = atoi(argv[i + 1]) != 0;
            else if (std::string(argv[i]) == "-skipSwapStarDist")
                config.skipSwapStarDist = atoi(argv[i + 1]) != 0;
            else if (std::string(argv[i])
                     == "-circleSectorOverlapToleranceDegrees")
                config.circleSectorOverlapToleranceDegrees = atoi(argv[i + 1]);
            else if (std::string(argv[i]) == "-minCircleSectorSizeDegrees")
                config.minCircleSectorSizeDegrees = atoi(argv[i + 1]);
        }

        return Params(config);
    }

    // Printing information to command line about how to use the code
    static void displayHelp()
    {
        std::cout << std::endl;
        std::cout << "-------------------------------------------------- "
                     "HGS-CVRPTW algorithm (2022) "
                     "-----------------------------------------"
                  << std::endl;
        std::cout << "Call with: ./genvrp instancePath solPath [-it nbIter] "
                     "[-t myCPUtime] [-seed mySeed] [-veh nbVehicles]\n";
        std::cout << std::endl;
        std::cout << "[-it <int>] sets a maximum number of iterations without "
                     "improvement. Defaults to 20,000\n";
        std::cout << "[-t <int>] sets a time limit in seconds. Defaults to "
                     "infinity\n";
        std::cout << "[-seed <int>] sets a fixed seed. Defaults to 0\n";
        std::cout << "[-veh <int>] sets a prescribed fleet size. Otherwise a "
                     "reasonable UB on the the fleet size is calculated\n";
        std::cout << std::endl;
        std::cout << "Additional Arguments:                                    "
                  << std::endl;
        std::cout << "[-useDynamicParameters <int>] sets when dynamic "
                     "parameters are used based on instance "
                     "attributes. It can be 0 or 1.     "
                  << std::endl;
        std::cout << "                              Defaults to 0              "
                  << std::endl;
        std::cout << "[-nbGranular <int>] Granular search parameter, limits "
                     "the number of moves in the RI local "
                     "search. Defaults to 40        "
                  << std::endl;
        std::cout << "[-fractionGeneratedNearest <double>] sets proportion of "
                     "individuals constructed by "
                     "nearest-first. Defaults to 0.05      "
                  << std::endl;
        std::cout << "[-fractionGeneratedFurthest <double>] sets proportion of "
                     "individuals constructed by "
                     "furthest-first. Defaults to 0.05    "
                  << std::endl;
        std::cout << "[-fractionGeneratedSweep <double>] sets proportion of "
                     "individuals constructed by sweep. Defaults "
                     "to 0.05                "
                  << std::endl;
        std::cout << "[-fractionGeneratedRandomly <double>] sets proportion of "
                     "individuals constructed randomly. "
                     "Defaults to 0.85             "
                  << std::endl;
        std::cout << "[-minSweepFillPercentage <int>] sets the fill percentage "
                     "for the individuals constructed by "
                     "sweep. Defaults to 60       "
                  << std::endl;
        std::cout << "[-maxToleratedCapacityViolation <int>] sets the maximum "
                     "tolerated violation of the capacity "
                     "restriction. Defaults to 50 "
                  << std::endl;
        std::cout << "[-maxToleratedTimeWarp <int>] sets the maximum tolerated "
                     "time warp. Defaults to 100              "
                  << std::endl;
        std::cout << "[-initialTimeWarpPenalty <double>] sets the time warp "
                     "penalty to use at the start of the "
                     "algorithm. Defaults to 1.0     "
                  << std::endl;
        std::cout << "[-penaltyBooster <double>] sets the multipl. factor for "
                     "time warp and capacity penalties when no "
                     "feas. solutions.       "
                  << std::endl;
        std::cout << "                           Defaults to 2.0               "
                  << std::endl;
        std::cout << "[-useSymmetricCorrelatedVertices <bool>] sets when "
                     "correlation matrix is symmetric. It can be 0 "
                     "or 1. Defaults to 0     "
                  << std::endl;
        std::cout << "[-doRepeatUntilTimeLimit <bool>] sets when to repeat the "
                     "algorithm when max nr of iter is "
                     "reached, but time limit is not"
                  << std::endl;
        std::cout << "                                 reached. It can be 0 or "
                     "1. Defaults to 1                        "
                  << std::endl;
        std::cout << "[-minimumPopulationSize <int>] sets the minimum "
                     "population size. Defaults to 25                  "
                  << std::endl;
        std::cout << "[-generationSize <int>] sets the number of solutions "
                     "created before reaching the maximum "
                     "population size. Defaults to 40"
                  << std::endl;
        std::cout << "[-nbElite <int>] sets the number of elite individuals. "
                     "Defaults to 4                             "
                  << std::endl;
        std::cout << "[-nbClose <int>] sets the number of closest individuals "
                     "when calculating diversity contribution. "
                     "Defaults to 5          "
                  << std::endl;
        std::cout << "[-targetFeasible <double>] sets proportion of number of "
                     "feasible individuals, used for penalty "
                     "params adaptation.       "
                  << std::endl;
        std::cout << "                           Defaults to 0.2               "
                  << std::endl;
        std::cout << "[-repairProbability <int>] sets the repair probability "
                     "if individual is infeasible after local "
                     "search. Defaults to 50   "
                  << std::endl;
        std::cout << "[-growNbGranularAfterNonImprovementIterations <int>] "
                     "sets the number of iterations without "
                     "improvements after which     "
                  << std::endl;
        std::cout << "                                                     the "
                     "nbGranular is grown. Defaults to 5000   "
                  << std::endl;
        std::cout << "[-growNbGranularAfterIterations <int>] sets the number "
                     "of iteration after which the nbGranular "
                     "is grown. Defaults to 0  "
                  << std::endl;
        std::cout << "[-growNbGranularSize <int>] sets the number nbGranular "
                     "is increase by. Defaults to 0             "
                  << std::endl;
        std::cout << "[-growPopulationAfterNonImprovementIterations <int>] "
                     "sets the number of iterations without "
                     "improvements after which     "
                  << std::endl;
        std::cout << "                                                     the "
                     "minimumPopulationSize is grown. "
                     "Defaults to 5000               "
                  << std::endl;
        std::cout << "[-growPopulationAfterIterations <int>] sets the number "
                     "of iteration after which "
                     "minimumPopulationSize is grown.         "
                  << std::endl;
        std::cout << "                                       Defaults to 0     "
                  << std::endl;
        std::cout << "[-growPopulationSize <int>] sets the number "
                     "minimumPopulationSize is increase by. Defaults to 0  "
                  << std::endl;
        std::cout << "[-intensificationProbabilityLS <int>] sets the "
                     "probability intensification moves are performed "
                     "during LS. Defaults to 15"
                  << std::endl;
        std::cout << "[-diversityWeight <double>] sets the weight for "
                     "diversity criterium, if 0, weight is "
                     "1-nbElite/populationSize.          "
                  << std::endl;
        std::cout << "                            Defaults to 0.0              "
                  << std::endl;
        std::cout << "[-useSwapStarTW <bool>] sets when to use time windows "
                     "swap star. It can be 0 or 1. Defaults to 1 "
                  << std::endl;
        std::cout << "[-skipSwapStarDist <bool>] sets when to skip normal swap "
                     "star based on distance. It can be 0 or "
                     "1. Defaults to 0        "
                  << std::endl;
        std::cout << "[-circleSectorOverlapToleranceDegrees <int>] sets the "
                     "margin to take (in degrees 0 - 359) to "
                     "determine overlap of circle"
                  << std::endl;
        std::cout << "                                             sectors for "
                     "SWAP*. Defaults to 0                    "
                  << std::endl;
        std::cout << "[-minCircleSectorSizeDegrees <int>] sets the minimum "
                     "size (in degrees 0 - 359) for circle "
                     "sectors such that even small  "
                  << std::endl;
        std::cout << "                                    circle sectors have "
                     "'overlap'. Defaults to 15                "
                  << std::endl;
    };
};

#endif
