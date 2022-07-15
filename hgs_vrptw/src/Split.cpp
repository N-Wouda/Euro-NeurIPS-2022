#include "Split.h"

#include "Individual.h"
#include "Params.h"

#include <cmath>
#include <deque>
#include <string>
#include <vector>

void Split::generalSplit(Individual *indiv, int nbMaxVehicles)
{
    // Do not apply Split with fewer vehicles than the trivial (LP) bin packing
    // bound
    maxVehicles = std::max(nbMaxVehicles,
                           static_cast<int>(std::ceil(
                               params->totalDemand / params->vehicleCapacity)));

    // Initialization of the data structures for the Linear Split algorithm
    // Direct application of the code located at
    // https://github.com/vidalt/Split-Library Loop over all clients, excluding
    // the depot
    for (int i = 1; i <= params->nbClients; i++)
    {
        // Store all information on clientSplits (use tourChrom[i-1] since the
        // depot is not included in tourChrom)
        cliSplit[i].demand = params->cli[indiv->tourChrom[i - 1]].demand;
        cliSplit[i].serviceTime
            = params->cli[indiv->tourChrom[i - 1]].serviceDuration;
        cliSplit[i].d0_x = params->timeCost.get(0, indiv->tourChrom[i - 1]);
        cliSplit[i].dx_0 = params->timeCost.get(indiv->tourChrom[i - 1], 0);

        // The distance to the next client is INT_MIN for the last client
        auto const dist = params->timeCost.get(indiv->tourChrom[i - 1],
                                               indiv->tourChrom[i]);
        cliSplit[i].dnext = i < params->nbClients ? dist : INT_MIN;

        // Store cumulative data on the demand, service time, and distance
        sumLoad[i] = sumLoad[i - 1] + cliSplit[i].demand;
        sumService[i] = sumService[i - 1] + cliSplit[i].serviceTime;
        sumDistance[i] = sumDistance[i - 1] + cliSplit[i - 1].dnext;
    }

    // Build up the rest of the Individual structure
    splitSimple(indiv);
    indiv->evaluateCompleteCost();
}

int Split::splitSimple(Individual *indiv)
{
    // Reinitialize the potential structure
    std::fill(potential[0].begin(), potential[0].end(), 1.e30);

    // The duration is not constrained here. This runs in O(n)
    // Create a queue of size nbClients + 1, where the first node is 0 (the
    // depot)
    auto deq = std::deque<int>(params->nbClients + 1);
    deq.push_front(0);

    // Loop over all clients, excluding the depot
    for (int i = 1; i <= params->nbClients; i++)
    {
        // The front (which is the depot in the first loop) is the best
        // predecessor for i
        potential[0][i] = propagate(deq.front(), i, 0);
        pred[0][i] = deq.front();

        // Check if i is not the last client
        if (i < params->nbClients)
        {
            // If i is not dominated by the last of the pile
            if (!dominates(deq.back(), i, 0))
            {
                // Then i will be inserted, need to remove whoever is
                // dominated by i
                while (!deq.empty() && dominatesRight(deq.back(), i, 0))
                    deq.pop_back();

                deq.push_back(i);
            }

            // Check iteratively if front is dominated by the next front
            while (deq.size() > 1
                   && propagate(deq.front(), i + 1, 0)
                          > propagate(deq[1], i + 1, 0) - MY_EPSILON)
            {
                deq.pop_front();
            }
        }
    }

    // Check if the cost of the last client is still very large. In that case,
    // the Split algorithm did not reach the last client
    if (potential[0][params->nbClients] > 1.e29)
        throw std::string("ERROR : no Split solution has been propagated until "
                          "the last node");

    // Filling the routeChrom structure
    // First clear some routeChrom vectors. In practice, maxVehicles equals
    // nbVehicles. Then, this loop is not needed and the next loop starts at the
    // last index of routeChrom
    for (int k = params->nbVehicles - 1; k >= maxVehicles; k--)
        indiv->routeChrom[k].clear();

    // Loop over all vehicles, clear the route, get the predecessor and create a
    // new routeChrom for that route
    int end = params->nbClients;
    for (int k = maxVehicles - 1; k >= 0; k--)
    {
        // Clear the corresponding routeChrom
        indiv->routeChrom[k].clear();

        // Loop from the begin to the end of the route corresponding to this
        // vehicle
        int begin = pred[0][end];
        for (int ii = begin; ii < end; ii++)
            indiv->routeChrom[k].push_back(indiv->tourChrom[ii]);

        end = begin;
    }

    // Return OK in case the Split algorithm reached the beginning of the routes
    return (end == 0);
}

Split::Split(Params *params) : params(params)
{
    // Initialize structures for the Linear Split
    cliSplit = std::vector<ClientSplit>(params->nbClients + 1);
    sumDistance = std::vector<int>(params->nbClients + 1, 0);
    sumLoad = std::vector<int>(params->nbClients + 1, 0);
    sumService = std::vector<int>(params->nbClients + 1, 0);
    potential = std::vector<std::vector<double>>(
        params->nbVehicles + 1,
        std::vector<double>(params->nbClients + 1, 1.e30));
    pred = std::vector<std::vector<int>>(
        params->nbVehicles + 1, std::vector<int>(params->nbClients + 1, 0));
}
