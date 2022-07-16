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

#include "Config.h"

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
    //		0) The path to the genvrp executable to run
    //		1) The path to the instance to consider
    //		2) The path to the file where the solution will be stored
    //      .) Possibly combinations of command line argument descriptions with
    //         their value (counted as 2 per argument in argc)
    CommandLine(int argc, char **argv) : argc(argc), argv(argv)
    {
        // Check if the number of arguments is odd and at least three, since
        // the two paths (+program name) should at least be given
        if (argc % 2 != 1 || argc < 3)
            throw std::invalid_argument("Incorrect number of arguments");
    }

    [[nodiscard]] char const *instPath() const { return argv[1]; }

    [[nodiscard]] char const *solPath() const { return argv[2]; }

    // Extracts run configurations from command line arguments
    Config parse()
    {
        Config config;

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

        return config;
    }
};

#endif
