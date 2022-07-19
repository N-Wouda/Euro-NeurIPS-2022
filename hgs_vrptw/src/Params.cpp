#include "Params.h"

#include "CircleSector.h"
#include "Matrix.h"
#include "XorShift128.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <set>
#include <string>
#include <vector>

Params::Params(Config &config, std::string const &instPath) : config(config)
{
    nbVehicles = config.nbVeh;

    // Initialize some parameter values
    std::string content, content2, content3;
    int serviceTimeData = 0;
    int node;
    bool hasServiceTimeSection = false;
    nbClients = 0;
    int totalDemand = 0;
    int maxDemand = 0;
    vehicleCapacity = INT_MAX;

    // Read INPUT dataset
    std::ifstream inputFile(instPath);

    if (!inputFile)
        throw std::invalid_argument("Impossible to open file: " + instPath);

    // Read the instance name from the first line and remove it
    getline(inputFile, content);

    // Read the next lines
    getline(inputFile,
            content);             // "Empty line" or "NAME : {instance_name}"
    getline(inputFile, content);  // VEHICLE or "COMMENT: {}"

    // Check if the next line has "VEHICLE"
    if (content.substr(0, 7) == "VEHICLE")
    {
        // Get the number of vehicles and the capacity of the vehicles
        getline(inputFile, content);  // NUMBER    CAPACITY
        inputFile >> nbVehicles >> vehicleCapacity;

        // Skip the next four lines
        getline(inputFile, content);
        getline(inputFile, content);
        getline(inputFile, content);
        getline(inputFile, content);

        // Create a vector where all information on the Clients can be
        // stored and loop over all information in the file
        cli = std::vector<Client>(1001);
        nbClients = 0;
        while (inputFile >> node)
        {
            // Store all the information of the next client
            cli[nbClients].custNum = node;
            inputFile >> cli[nbClients].x >> cli[nbClients].y
                >> cli[nbClients].demand >> cli[nbClients].twEarly
                >> cli[nbClients].twLate >> cli[nbClients].servDur;

            // Scale coordinates by factor 10, later the distances will be
            // rounded so we optimize with 1 decimal distances
            cli[nbClients].x *= 10;
            cli[nbClients].y *= 10;
            cli[nbClients].twEarly *= 10;
            cli[nbClients].twLate *= 10;
            cli[nbClients].servDur *= 10;
            cli[nbClients].angle = CircleSector::positive_mod(
                static_cast<int>(32768.
                                 * atan2(cli[nbClients].y - cli[0].y,
                                         cli[nbClients].x - cli[0].x)
                                 / M_PI));

            // Keep track of the max demand, the total demand, and the
            // number of clients
            if (cli[nbClients].demand > maxDemand)
            {
                maxDemand = cli[nbClients].demand;
            }
            totalDemand += cli[nbClients].demand;
            nbClients++;
        }

        // Reduce the size of the vector of clients if possible
        cli.resize(nbClients);

        // Don't count depot as client
        nbClients--;

        // Check if the required service and the start of the time window of
        // the depot are both zero
        if (cli[0].twEarly != 0)
        {
            throw std::runtime_error("Depot time window should start at 0");
        }
        if (cli[0].servDur != 0)
        {
            throw std::runtime_error("Depot service duration should be 0");
        }
    }
    else
    {
        // CVRP or VRPTW according to VRPLib format
        for (inputFile >> content; content != "EOF"; inputFile >> content)
        {
            // Read the dimension of the problem (the number of clients)
            if (content == "DIMENSION")
            {
                // Need to substract the depot from the number of nodes
                inputFile >> content2 >> nbClients;
                nbClients--;
            }
            // Read the type of edge weights
            else if (content == "EDGE_WEIGHT_TYPE")
            {
                inputFile >> content2 >> content3;
            }
            else if (content == "EDGE_WEIGHT_FORMAT")
            {
                inputFile >> content2 >> content3;
                if (content3 != "FULL_MATRIX")
                {
                    throw std::runtime_error(
                        "EDGE_WEIGHT_FORMAT only supports FULL_MATRIX");
                }
            }
            else if (content == "CAPACITY")
            {
                inputFile >> content2 >> vehicleCapacity;
            }
            else if (content == "VEHICLES" || content == "SALESMAN")
            {
                inputFile >> content2 >> nbVehicles;
            }
            // Read the data on the service time (used when the service time
            // is constant for all clients)
            else if (content == "SERVICE_TIME")
            {
                inputFile >> content2 >> serviceTimeData;
            }
            // Read the edge weights of an explicit distance matrix
            else if (content == "EDGE_WEIGHT_SECTION")
            {
                timeCost = Matrix(nbClients + 1);
                for (int i = 0; i <= nbClients; i++)
                {
                    for (int j = 0; j <= nbClients; j++)
                    {
                        // Keep track of the largest distance between two
                        // clients (or the depot)
                        int cost;
                        inputFile >> cost;
                        timeCost.set(i, j, cost);
                    }
                }
            }
            else if (content == "NODE_COORD_SECTION")
            {
                // Reading client coordinates
                cli = std::vector<Client>(nbClients + 1);
                for (int i = 0; i <= nbClients; i++)
                {
                    inputFile >> cli[i].custNum >> cli[i].x >> cli[i].y;

                    // Check if the clients are in order
                    if (cli[i].custNum != i + 1)
                    {
                        throw std::runtime_error("Coordinates are not in "
                                                 "order of clients");
                    }

                    cli[i].custNum--;
                    cli[i].angle = CircleSector::positive_mod(static_cast<int>(
                        32768. * atan2(cli[i].y - cli[0].y, cli[i].x - cli[0].x)
                        / M_PI));
                }
            }
            // Read the demand of each client (including the depot, which
            // should have demand 0)
            else if (content == "DEMAND_SECTION")
            {
                for (int i = 0; i <= nbClients; i++)
                {
                    int clientNr = 0;
                    inputFile >> clientNr >> cli[i].demand;

                    // Check if the clients are in order
                    if (clientNr != i + 1)
                    {
                        throw std::runtime_error("Clients are not in order"
                                                 " in the list of demands");
                    }

                    // Keep track of the max and total demand
                    if (cli[i].demand > maxDemand)
                    {
                        maxDemand = cli[i].demand;
                    }
                    totalDemand += cli[i].demand;
                }
                // Check if the depot has demand 0
                if (cli[0].demand != 0)
                {
                    throw std::runtime_error(
                        "Depot demand is not zero, but is instead: "
                        + std::to_string(cli[0].servDur));
                }
            }
            else if (content == "DEPOT_SECTION")
            {
                inputFile >> content2 >> content3;
                if (content2 != "1")
                {
                    throw std::runtime_error("Expected depot index 1 "
                                             "instead of "
                                             + content2);
                }
            }
            else if (content == "SERVICE_TIME_SECTION")
            {
                for (int i = 0; i <= nbClients; i++)
                {
                    int clientNr = 0;
                    inputFile >> clientNr >> cli[i].servDur;

                    // Check if the clients are in order
                    if (clientNr != i + 1)
                    {
                        throw std::runtime_error("Service times are not "
                                                 "in client order");
                    }
                }
                // Check if the service duration of the depot is 0
                if (cli[0].servDur != 0)
                {
                    throw std::runtime_error(
                        "Service duration for depot should be 0");
                }
                hasServiceTimeSection = true;
            }
            else if (content == "RELEASE_TIME_SECTION")
            {
                for (int i = 0; i <= nbClients; i++)
                {
                    int clientNr = 0;
                    inputFile >> clientNr >> cli[i].releaseTime;

                    // Check if the clients are in order
                    if (clientNr != i + 1)
                    {
                        throw std::runtime_error("Release times are not in"
                                                 " client order");
                    }
                }
                // Check if the service duration of the depot is 0
                if (cli[0].releaseTime != 0)
                {
                    throw std::runtime_error(
                        "Release time for depot should be 0");
                }
            }
            // Read the time windows of all the clients (the depot should
            // have a time window from 0 to max)
            else if (content == "TIME_WINDOW_SECTION")
            {
                for (int i = 0; i <= nbClients; i++)
                {
                    int clientNr = 0;
                    inputFile >> clientNr >> cli[i].twEarly >> cli[i].twLate;

                    // Check if the clients are in order
                    if (clientNr != i + 1)
                    {
                        throw std::runtime_error("Time windows are not in "
                                                 "client order");
                    }
                }

                // Check the start of the time window of the depot
                if (cli[0].twEarly != 0)
                {
                    throw std::runtime_error(
                        "Time window for depot should start at 0");
                }
            }
            else
            {
                throw std::runtime_error("Unexpected data in input file: "
                                         + content);
            }
        }

        if (!hasServiceTimeSection)
        {
            for (int i = 0; i <= nbClients; i++)
            {
                cli[i].servDur = (i == 0) ? 0 : serviceTimeData;
            }
        }

        if (nbClients <= 0)
        {
            throw std::runtime_error("Number of nodes is undefined");
        }
        if (vehicleCapacity == INT_MAX)
            throw std::runtime_error("Vehicle capacity is undefined");
    }

    // Default initialization if the number of vehicles has not been provided by
    // the user
    if (nbVehicles == INT_MAX)
    {
        // Safety margin: 30% + 3 more vehicles than the trivial bin packing LB
        nbVehicles = static_cast<int>(
            std::ceil(1.3 * totalDemand / vehicleCapacity) + 3.);
    }
    else if (nbVehicles == -1)  // unlimited
    {
        nbVehicles = nbClients;
    }

    // When dynamic parameters have to be used, set more parameter values
    if (config.useDynamicParameters)
    {
        // Determine categories of instances based on number of stops/route and
        // whether it has large time windows Calculate an upper bound for the
        // number of stops per route based on capacities
        double stopsPerRoute = vehicleCapacity / (totalDemand / nbClients);
        // Routes are large when more than 25 stops per route
        bool hasLargeRoutes = stopsPerRoute > 25;

        // Get the time horizon (by using the time window of the depot)
        int horizon = cli[0].twLate - cli[0].twEarly;
        int nbLargeTW = 0;

        // Loop over all clients (excluding the depot) and count the amount of
        // large time windows (greater than 0.7*horizon)
        for (int i = 1; i <= nbClients; i++)
            if (cli[i].twLate - cli[i].twEarly > 0.7 * horizon)
                nbLargeTW++;

        bool hasLargeTW = nbLargeTW > 0;

        // Set the parameter values based on the characteristics of the instance
        if (hasLargeRoutes)
        {
            config.nbGranular = 40;
            // Grow neighborhood and population size
            config.growNbGranularAfterIterations = 10000;
            config.growNbGranularSize = 5;
            config.growPopulationAfterIterations = 10000;
            config.growPopulationSize = 5;
            // Intensify occasionally
            config.intensificationProbabilityLS = 15;
        }
        else
        {
            // Grow population size only
            // config.growNbGranularAfterIterations = 10000;
            // config.growNbGranularSize = 5;
            if (hasLargeTW)
            {
                // Smaller neighbourhood so iterations are faster
                // So take more iterations before growing population
                config.nbGranular = 20;
                config.growPopulationAfterIterations = 20000;
            }
            else
            {
                config.nbGranular = 40;
                config.growPopulationAfterIterations = 10000;
            }
            config.growPopulationSize = 5;
            // Intensify always
            config.intensificationProbabilityLS = 100;
        }
    }

    int maxDist = timeCost.max();

    // Calculate, for all vertices, the correlation for the nbGranular closest
    // vertices
    setCorrelatedVertices();

    // Safeguards to avoid possible numerical instability in case of instances
    // containing arbitrarily small or large numerical values
    if (maxDist < 0.1 || maxDist > 100000)
    {
        throw std::runtime_error(
            "The distances are of very small or large scale. This could impact "
            "numerical stability. Please rescale the dataset and run again.");
    }
    if (maxDemand < 0.1 || maxDemand > 100000)
    {
        throw std::runtime_error(
            "The demand quantities are of very small or large scale. This "
            "could impact numerical stability. Please rescale the dataset and "
            "run again.");
    }
    if (nbVehicles < std::ceil(totalDemand / vehicleCapacity))
    {
        throw std::runtime_error(
            "Fleet size is insufficient to service the considered clients.");
    }

    // A reasonable scale for the initial values of the penalties
    penaltyCapacity = std::max(
        0.1, std::min(1000., static_cast<double>(maxDist) / maxDemand));

    // Initial parameter values of these two parameters are not argued
    penaltyWaitTime = 0.;
    penaltyTimeWarp = config.initialTimeWarpPenalty;
}

Params::Params(Config &config,
               std::vector<std::pair<int, int>> const &coords,
               std::vector<int> const &demands,
               int vehicleCap,
               std::vector<std::pair<int, int>> const &timeWindows,
               std::vector<int> const &servDurs,
               std::vector<std::vector<int>> const &dist)
    : config(config), nbClients(coords.size() - 1), vehicleCapacity(vehicleCap)
{
    int totalDemand = std::accumulate(demands.begin(), demands.end(), 0);
    int maxDemand = *std::max_element(demands.begin(), demands.end());

    // Number of vehicles: 30% above LP bin packing heuristic, and three more
    // just in case.
    auto const vehicleMargin = std::ceil(1.3 * totalDemand / vehicleCapacity);
    nbVehicles = static_cast<int>(vehicleMargin) + 3;

    timeCost = Matrix(dist.size());

    for (size_t i = 0; i != dist.size(); ++i)
        for (size_t j = 0; j != dist[i].size(); ++j)
            timeCost.set(i, j, dist[i][j]);

    int maxDist = timeCost.max();

    // A reasonable scale for the initial values of the penalties
    penaltyCapacity = std::max(
        0.1, std::min(1000., static_cast<double>(maxDist) / maxDemand));

    // Initial parameter values of these two parameters are not argued
    penaltyWaitTime = 0.;
    penaltyTimeWarp = config.initialTimeWarpPenalty;

    cli = std::vector<Client>(nbClients + 1);

    for (size_t idx = 0; idx <= static_cast<size_t>(nbClients); ++idx)
    {
        auto const angle = CircleSector::positive_mod(
            static_cast<int>(32768.
                             * atan2(cli[nbClients].y - coords[idx].second,
                                     cli[nbClients].x - coords[idx].first)
                             / M_PI));

        cli[idx] = {static_cast<int>(idx + 1),
                    coords[idx].first,
                    coords[idx].second,
                    servDurs[idx],
                    demands[idx],
                    timeWindows[idx].first,
                    timeWindows[idx].second,
                    0,  // TODO no release times?
                    angle};
    }

    setCorrelatedVertices();
}

void Params::setCorrelatedVertices()
{
    // See Vidal 2012, HGS for VRPTW. Multiplied by 10 for integer arithmetic.
    int const weightWaitTime = 2;
    int const weightTimeWarp = 10;

    auto proximities
        = std::vector<std::vector<std::pair<int, int>>>(nbClients + 1);

    for (int i = 1; i <= nbClients; i++)  // exclude depot
    {
        auto &proximity = proximities[i];

        for (int j = 1; j <= nbClients; j++)  // exclude depot
        {
            if (i == j)  // exclude the current client
                continue;

            // Compute proximity using Eq. 4 in Vidal 2012
            int const time = timeCost.get(i, j);

            int const first
                = weightWaitTime
                      * std::max(cli[j].twEarly - time - cli[i].servDur
                                     - cli[i].twLate,
                                 0)
                  + weightTimeWarp
                        * std::max(cli[i].twEarly + cli[i].servDur + time
                                       - cli[j].twLate,
                                   0);
            int const second
                = weightWaitTime
                      * std::max(cli[i].twEarly - time - cli[j].servDur
                                     - cli[j].twLate,
                                 0)
                  + weightTimeWarp
                        * std::max(cli[j].twEarly + cli[j].servDur + time
                                       - cli[i].twLate,
                                   0);

            proximity.emplace_back(time + std::min(first, second), j);
        }

        std::sort(proximity.begin(), proximity.end());
    }

    correlatedVertices = std::vector<std::vector<int>>(nbClients + 1);

    // First create a set of correlated vertices for each vertex (where the
    // depot is not taken into account)
    auto set = std::vector<std::set<int>>(nbClients + 1);

    for (int i = 1; i <= nbClients; i++)  // again exclude depot
    {
        auto &orderProximity = proximities[i];

        // Loop over all clients (taking into account the max number of clients
        // and the granular restriction)
        for (int j = 0; j < std::min(config.nbGranular, nbClients - 1); j++)
        {
            // If i is correlated with j, then j should be correlated with i
            // (unless we have asymmetric problem with time windows) Insert
            // vertices in set, in the order of
            // orderProximity, where .second is used since the first index
            // correponds to the depot
            set[i].insert(orderProximity[j].second);

            // For symmetric problems, set the other entry to the same value
            if (config.useSymmetricCorrelatedVertices)
                set[orderProximity[j].second].insert(i);
        }
    }

    // Now, fill the vector of correlated vertices, using set
    for (int i = 1; i <= nbClients; i++)
        for (int x : set[i])
            correlatedVertices[i].push_back(x);
}
