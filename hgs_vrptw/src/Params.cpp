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
    startWallClockTime = std::chrono::system_clock::now();
    startCPUTime = std::clock();

    // Convert the circle sector parameters from degrees ([0,359]) to [0,65535]
    // to allow for faster calculations
    circleSectorOverlapTolerance = static_cast<int>(
        config.circleSectorOverlapToleranceDegrees / 360. * 65536);
    minCircleSectorSize
        = static_cast<int>(config.minCircleSectorSizeDegrees / 360. * 65536);

    // Initialize some parameter values
    std::string content, content2, content3;
    int serviceTimeData = 0;
    int node;
    bool hasServiceTimeSection = false;
    nbClients = 0;
    totalDemand = 0;
    maxDemand = 0;
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
            inputFile >> cli[nbClients].coordX >> cli[nbClients].coordY
                >> cli[nbClients].demand >> cli[nbClients].earliestArrival
                >> cli[nbClients].latestArrival
                >> cli[nbClients].serviceDuration;

            // Scale coordinates by factor 10, later the distances will be
            // rounded so we optimize with 1 decimal distances
            cli[nbClients].coordX *= 10;
            cli[nbClients].coordY *= 10;
            cli[nbClients].earliestArrival *= 10;
            cli[nbClients].latestArrival *= 10;
            cli[nbClients].serviceDuration *= 10;
            cli[nbClients].polarAngle = CircleSector::positive_mod(
                static_cast<int>(32768.
                                 * atan2(cli[nbClients].coordY - cli[0].coordY,
                                         cli[nbClients].coordX - cli[0].coordX)
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
        if (cli[0].earliestArrival != 0)
        {
            throw std::runtime_error("Depot time window should start at 0");
        }
        if (cli[0].serviceDuration != 0)
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
                if (content3 == "EXPLICIT")
                {
                    isExplicitDistanceMatrix = true;
                }
            }
            else if (content == "EDGE_WEIGHT_FORMAT")
            {
                inputFile >> content2 >> content3;
                if (!isExplicitDistanceMatrix)
                {
                    throw std::runtime_error(
                        "EDGE_WEIGHT_FORMAT can only be used "
                        "with EDGE_WEIGHT_TYPE : EXPLICIT");
                }

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
                if (!isExplicitDistanceMatrix)
                {
                    throw std::runtime_error(
                        "EDGE_WEIGHT_SECTION can only be used with "
                        "EDGE_WEIGHT_TYPE : EXPLICIT");
                }

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
                    inputFile >> cli[i].custNum >> cli[i].coordX
                        >> cli[i].coordY;

                    // Check if the clients are in order
                    if (cli[i].custNum != i + 1)
                    {
                        throw std::runtime_error("Coordinates are not in "
                                                 "order of clients");
                    }

                    cli[i].custNum--;
                    cli[i].polarAngle = CircleSector::positive_mod(
                        static_cast<int>(32768.
                                         * atan2(cli[i].coordY - cli[0].coordY,
                                                 cli[i].coordX - cli[0].coordX)
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
                        + std::to_string(cli[0].serviceDuration));
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
                    inputFile >> clientNr >> cli[i].serviceDuration;

                    // Check if the clients are in order
                    if (clientNr != i + 1)
                    {
                        throw std::runtime_error("Service times are not "
                                                 "in client order");
                    }
                }
                // Check if the service duration of the depot is 0
                if (cli[0].serviceDuration != 0)
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
                    inputFile >> clientNr >> cli[i].earliestArrival
                        >> cli[i].latestArrival;

                    // Check if the clients are in order
                    if (clientNr != i + 1)
                    {
                        throw std::runtime_error("Time windows are not in "
                                                 "client order");
                    }
                }

                // Check the start of the time window of the depot
                if (cli[0].earliestArrival != 0)
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
                cli[i].serviceDuration = (i == 0) ? 0 : serviceTimeData;
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
        int horizon = cli[0].latestArrival - cli[0].earliestArrival;
        int nbLargeTW = 0;

        // Loop over all clients (excluding the depot) and count the amount of
        // large time windows (greater than 0.7*horizon)
        for (int i = 1; i <= nbClients; i++)
            if (cli[i].latestArrival - cli[i].earliestArrival > 0.7 * horizon)
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

    if (!isExplicitDistanceMatrix)
    {
        // Calculation of the distance matrix
        timeCost = Matrix(nbClients + 1);
        // Loop over all clients (including the depot)
        for (int i = 0; i <= nbClients; i++)
        {
            // Set the diagonal element to zero (travel to itself)
            timeCost.set(i, i, 0);
            // Loop over all other clients
            for (int j = i + 1; j <= nbClients; j++)
            {
                // Calculate Euclidian distance d
                double d = std::sqrt((cli[i].coordX - cli[j].coordX)
                                         * (cli[i].coordX - cli[j].coordX)
                                     + (cli[i].coordY - cli[j].coordY)
                                           * (cli[i].coordY - cli[j].coordY));
                // Integer truncation
                int cost = static_cast<int>(d);

                // Save the distances in the matrix
                timeCost.set(i, j, cost);
                timeCost.set(j, i, cost);
            }
        }
    }

    maxDist = timeCost.max();

    // Compute order proximities once
    orderProximities
        = std::vector<std::vector<std::pair<double, int>>>(nbClients + 1);
    // Loop over all clients (excluding the depot)
    for (int i = 1; i <= nbClients; i++)
    {
        // Remove all elements from the vector
        auto &orderProximity = orderProximities[i];
        orderProximity.clear();

        // Loop over all clients (excluding the depot and the specific client
        // itself)
        for (int j = 1; j <= nbClients; j++)
        {
            if (i != j)
            {
                // Compute proximity using Eq. 4 in Vidal 2012, and append at
                // the end of orderProximity
                const int timeIJ = timeCost.get(i, j);
                orderProximity.emplace_back(
                    timeIJ
                        + std::min(
                            proximityWeightWaitTime
                                    * std::max(cli[j].earliestArrival - timeIJ
                                                   - cli[i].serviceDuration
                                                   - cli[i].latestArrival,
                                               0)
                                + proximityWeightTimeWarp
                                      * std::max(cli[i].earliestArrival
                                                     + cli[i].serviceDuration
                                                     + timeIJ
                                                     - cli[j].latestArrival,
                                                 0),
                            proximityWeightWaitTime
                                    * std::max(cli[i].earliestArrival - timeIJ
                                                   - cli[j].serviceDuration
                                                   - cli[j].latestArrival,
                                               0)
                                + proximityWeightTimeWarp
                                      * std::max(cli[j].earliestArrival
                                                     + cli[j].serviceDuration
                                                     + timeIJ
                                                     - cli[i].latestArrival,
                                                 0)),
                    j);
            }
        }

        // Sort orderProximity (for the specific client)
        std::sort(orderProximity.begin(), orderProximity.end());
    }

    // Calculate, for all vertices, the correlation for the nbGranular closest
    // vertices
    SetCorrelatedVertices();

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

    // See Vidal 2012, HGS for VRPTW
    proximityWeightWaitTime = 0.2;
    proximityWeightTimeWarp = 1;
}

Params::Params(Config &config,
               std::vector<std::pair<int, int>> const &coords,
               std::vector<int> const &demands,
               int vehicleCap,
               std::vector<std::pair<int, int>> const &timeWindows,
               std::vector<int> const &servDurs,
               std::vector<std::vector<int>> const &dist)
    : config(config),
      isExplicitDistanceMatrix(true),
      nbClients(coords.size() - 1),
      nbVehicles(coords.size() - 1),  // TODO shrink?
      vehicleCapacity(vehicleCap)
{
    totalDemand = std::accumulate(demands.begin(), demands.end(), 0);
    maxDemand = *std::max_element(demands.begin(), demands.end());

    timeCost = Matrix(dist.size());

    for (size_t i = 0; i != dist.size(); ++i)
        for (size_t j = 0; j != dist[i].size(); ++j)
            timeCost.set(i, j, dist[i][j]);

    maxDist = timeCost.max();

    // A reasonable scale for the initial values of the penalties
    penaltyCapacity = std::max(
        0.1, std::min(1000., static_cast<double>(maxDist) / maxDemand));

    // Initial parameter values of these two parameters are not argued
    penaltyWaitTime = 0.;
    penaltyTimeWarp = config.initialTimeWarpPenalty;

    // See Vidal 2012, HGS for VRPTW
    proximityWeightWaitTime = 0.2;
    proximityWeightTimeWarp = 1;

    startWallClockTime = std::chrono::system_clock::now();
    startCPUTime = std::clock();

    // Convert the circle sector parameters from degrees ([0,359]) to [0,65535]
    // to allow for faster calculations
    circleSectorOverlapTolerance = static_cast<int>(
        config.circleSectorOverlapToleranceDegrees / 360. * 65536);
    minCircleSectorSize
        = static_cast<int>(config.minCircleSectorSizeDegrees / 360. * 65536);

    cli = std::vector<Client>(nbClients + 1);

    for (size_t idx = 0; idx <= static_cast<size_t>(nbClients); ++idx)
    {
        auto const angle = CircleSector::positive_mod(
            static_cast<int>(32768.
                             * atan2(cli[nbClients].coordY - coords[idx].second,
                                     cli[nbClients].coordX - coords[idx].first)
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

    // Compute order proximities once
    orderProximities
        = std::vector<std::vector<std::pair<double, int>>>(nbClients + 1);
    // Loop over all clients (excluding the depot)
    for (int i = 1; i <= nbClients; i++)
    {
        // Remove all elements from the vector
        auto &orderProximity = orderProximities[i];
        orderProximity.clear();

        // Loop over all clients (excluding the depot and the specific client
        // itself)
        for (int j = 1; j <= nbClients; j++)
        {
            if (i != j)
            {
                // Compute proximity using Eq. 4 in Vidal 2012, and append at
                // the end of orderProximity
                const int timeIJ = timeCost.get(i, j);
                orderProximity.emplace_back(
                    timeIJ
                        + std::min(
                            proximityWeightWaitTime
                                    * std::max(cli[j].earliestArrival - timeIJ
                                                   - cli[i].serviceDuration
                                                   - cli[i].latestArrival,
                                               0)
                                + proximityWeightTimeWarp
                                      * std::max(cli[i].earliestArrival
                                                     + cli[i].serviceDuration
                                                     + timeIJ
                                                     - cli[j].latestArrival,
                                                 0),
                            proximityWeightWaitTime
                                    * std::max(cli[i].earliestArrival - timeIJ
                                                   - cli[j].serviceDuration
                                                   - cli[j].latestArrival,
                                               0)
                                + proximityWeightTimeWarp
                                      * std::max(cli[j].earliestArrival
                                                     + cli[j].serviceDuration
                                                     + timeIJ
                                                     - cli[i].latestArrival,
                                                 0)),
                    j);
            }
        }

        // Sort orderProximity (for the specific client)
        std::sort(orderProximity.begin(), orderProximity.end());
    }

    SetCorrelatedVertices();
}

double Params::getElapsedTime() const
{
    if (config.useWallClockTime)
    {
        auto now = std::chrono::system_clock::now();
        std::chrono::duration<double> duration = now - startWallClockTime;
        return duration.count();
    }

    return static_cast<double>(std::clock() - startCPUTime) / CLOCKS_PER_SEC;
}

bool Params::isTimeLimitExceeded() const
{
    return getElapsedTime() >= config.timeLimit;
}

void Params::SetCorrelatedVertices()
{
    // Calculation of the correlated vertices for each client (for the granular
    // restriction)
    correlatedVertices = std::vector<std::vector<int>>(nbClients + 1);

    // First create a set of correlated vertices for each vertex (where the
    // depot is not taken into account)
    std::vector<std::set<int>> setCorrelatedVertices
        = std::vector<std::set<int>>(nbClients + 1);

    // Loop over all clients (excluding the depot)
    for (int i = 1; i <= nbClients; i++)
    {
        auto &orderProximity = orderProximities[i];

        // Loop over all clients (taking into account the max number of clients
        // and the granular restriction)
        for (int j = 0; j < std::min(config.nbGranular, nbClients - 1); j++)
        {
            // If i is correlated with j, then j should be correlated with i
            // (unless we have asymmetric problem with time windows) Insert
            // vertices in setCorrelatedVertices, in the order of
            // orderProximity, where .second is used since the first index
            // correponds to the depot
            setCorrelatedVertices[i].insert(orderProximity[j].second);

            // For symmetric problems, set the other entry to the same value
            if (config.useSymmetricCorrelatedVertices)
                setCorrelatedVertices[orderProximity[j].second].insert(i);
        }
    }

    // Now, fill the vector of correlated vertices, using setCorrelatedVertices
    for (int i = 1; i <= nbClients; i++)
        for (int x : setCorrelatedVertices[i])
            correlatedVertices[i].push_back(x);
}
