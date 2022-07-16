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

#include "Config.h"
#include "Matrix.h"
#include "XorShift128.h"

#include <chrono>
#include <climits>
#include <ctime>
#include <string>
#include <vector>

#define MY_EPSILON 0.00001  // Used to avoid numerical instabilities

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
    // TODO get rid of this object; turn it into ProblemData (and then have
    //  config as a separate object)
public:
    Config config;  // Stores all the parameter values

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

    // Get time elapsed since start of program
    [[nodiscard]] double getElapsedTime() const;

    // Whether time limit is exceeded
    [[nodiscard]] bool isTimeLimitExceeded() const;

    // Calculate, for all vertices, the correlation for the nbGranular closest
    // vertices
    void SetCorrelatedVertices();
};

#endif
