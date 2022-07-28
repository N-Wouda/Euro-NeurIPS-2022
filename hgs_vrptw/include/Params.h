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

    // Neighborhood restrictions: For each client, list of nearby clients (size
    // nbClients + 1, but nothing stored for the depot!)
    std::vector<std::vector<int>> neighbours;

    /**
     * Calculate, for all vertices, the correlation ('nearness') of the
     * nbGranular closest vertices.
     */
    void calculateNeighbours();

public:
    // TODO make members private

    Config const config;  // Stores all the parameter values

    double penaltyCapacity;  // Excess capacity penalty (per unit)
    double penaltyTimeWarp;  // Time warp penalty (per unit)

    int nbClients;        // Number of clients (excluding the depot)
    int nbVehicles;       // Number of vehicles
    int vehicleCapacity;  // Capacity limit

    std::vector<Client> clients;  // Client (+depot) information
    Matrix dist;                  // Distance matrix (+depot)

    /**
     * Returns the nbGranular clients nearest/closest to the passed-in client.
     */
    [[nodiscard]] std::vector<int> const &getNeighboursOf(size_t client) const
    {
        return neighbours[client];
    }

    /**
     * Constructs a Params object with the given configuration, and data read
     * from the given instance path.
     *
     * @param config   Configuration object.
     * @param instPath Path to the instance data.
     */
    Params(Config const &config, std::string const &instPath);

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
     * @param distMat      Distance matrix.
     * @param releases     Client release times.
     */
    Params(Config const &config,
           std::vector<std::pair<int, int>> const &coords,
           std::vector<int> const &demands,
           int vehicleCap,
           std::vector<std::pair<int, int>> const &timeWindows,
           std::vector<int> const &servDurs,
           std::vector<std::vector<int>> const &distMat,
           std::vector<int> const &releases);
};

#endif
