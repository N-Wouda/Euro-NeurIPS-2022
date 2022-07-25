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

#include <iosfwd>
#include <vector>

#define MY_EPSILON 0.00001  // Used to avoid numerical instabilities

// Class that stores all the parameters (from the command line) (in Config) and
// data of the instance needed by the algorithm
class Params
{
    // TODO get rid of this object; turn it into ProblemData (and then have
    //  Config as a separate object)

    // Structure of a Client, including its index, position, and all other
    // variables and parameters
    struct Client
    {
        int custNum;      // Index of the client
        int x;            // Coordinate X
        int y;            // Coordinate Y
        int servDur;      // Service duration
        int demand;       // Demand
        int twEarly;      // Earliest arrival (when using time windows)
        int twLate;       // Latest arrival (when using time windows)
        int releaseTime;  // Routes with this client cannot leave depot before
                          // this time

        // Polar angle of the client around the depot (starting at east, moving
        // counter-clockwise), measured in degrees and truncated for convenience
        int angle;
    };

public:
    // TODO make members private

    Config config;  // Stores all the parameter values

    // Penalty for one unit of capacity excess (adapted through the search)
    double penaltyCapacity;

    // Penalty for one unit time warp (adapted through the search)
    double penaltyTimeWarp;

    int nbClients;        // Number of clients (excluding the depot)
    int nbVehicles;       // Number of vehicles
    int vehicleCapacity;  // Capacity limit

    std::vector<Client> cli;  // Client (+depot) information
    Matrix timeCost;          // Distance matrix (+depot)

    // Neighborhood restrictions: For each client, list of nearby clients (size
    // nbClients + 1, but nothing stored for the depot!)
    std::vector<std::vector<int>> correlatedVertices;

    /**
     * Constructs a Params object with the given configuration, and data read
     * from the given instance path.
     *
     * @param config   Configuration object.
     * @param instPath Path to the instance data.
     */
    Params(Config &config, std::string const &instPath);

    /**
     * Constructs a Params object with the given configuration, and passed-in
     * data. Assumes the data contains the depot, such that each vector is one
     * longer than the number of clients.
     *
     * @param config       Configuration object.
     * @param coords       Coordinates as pairs of [x, y].
     * @param demands      Client demands.
     * @param vehicleCap   Vehicle capacity.
     * @param timeWindows  Time windows as pairs of [early, late].
     * @param servDurs     Service durations.
     * @param dist         Distance matrix.
     * @param releases     Client release times.
     */
    Params(Config &config,
           std::vector<std::pair<int, int>> const &coords,
           std::vector<int> const &demands,
           int vehicleCap,
           std::vector<std::pair<int, int>> const &timeWindows,
           std::vector<int> const &servDurs,
           std::vector<std::vector<int>> const &dist,
           std::vector<int> const &releases);

    // Calculate, for all vertices, the correlation for the nbGranular closest
    // vertices
    void setCorrelatedVertices();
};

#endif
