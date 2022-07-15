/*MIT License

Copyright(c) 2020 Thibaut Vidal

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

#ifndef SPLIT_H
#define SPLIT_H

#include "Individual.h"
#include "Params.h"

#include <cassert>
#include <vector>

// Structure representing a client used in the Split algorithm
struct ClientSplit
{
    int demand;       // The demand of the client
    int serviceTime;  // The service duration of the client
    int d0_x;         // The distance from the depot to the client
    int dx_0;         // The distance from the client to the depot
    int dnext;        // The distance from the client to the next client

    // Constructor, initializing everything with zero
    ClientSplit() : demand(0), serviceTime(0), d0_x(0), dx_0(0), dnext(0) {}
};

class Split
{
    Params *params;
    int maxVehicles;

    // Used by the linear split algorithm (size nbClients + 1); vector of all
    // client splits.
    std::vector<ClientSplit> cliSplit;

    // potential[0][t] is the shortest path cost from 0 to t;  goal is to
    // minimise the potential
    std::vector<std::vector<double>> potential;

    // The next variable pred stores the client starting the route of a given
    // client. So pred[k] is the client starting the route where k is also in.
    // Index of the predecessor in an optimal path
    std::vector<std::vector<int>> pred;

    // Cumulative distance. sumDistance[i] for i > 1 contains the sum of
    // distances : sum_{k=1}^{i-1} d_{k,k+1}
    std::vector<int> sumDistance;

    // Cumulative demand. sumLoad[i] for i >= 1 contains the sum of loads :
    // sum_{k=1}^{i} q_k
    std::vector<int> sumLoad;

    // Cumulative service time. sumService[i] for i >= 1 contains the sum of
    // service time : sum_{k=1}^{i} s_k
    std::vector<int> sumService;

    // To be called with i < j only
    // Computes the cost of propagating the label i until j
    inline double propagate(int i, int j, int k)
    {
        assert(i < j);
        return potential[k][i] + sumDistance[j] - sumDistance[i + 1]
               + cliSplit[i + 1].d0_x + cliSplit[j].dx_0
               + params->penaltyCapacity
                     * std::max(
                         sumLoad[j] - sumLoad[i] - params->vehicleCapacity, 0);
    }

    // Tests if i dominates j as a predecessor for all nodes x >= j+1
    // We assume that i < j
    inline bool dominates(int i, int j, int k)
    {
        assert(i < j);
        return potential[k][j] + cliSplit[j + 1].d0_x
               > potential[k][i] + cliSplit[i + 1].d0_x + sumDistance[j + 1]
                     - sumDistance[i + 1]
                     + params->penaltyCapacity * (sumLoad[j] - sumLoad[i]);
    }

    // Tests if j dominates i as a predecessor for all nodes x >= j+1
    // We assume that i < j
    inline bool dominatesRight(int i, int j, int k)
    {
        assert(i < j);
        return potential[k][j] + cliSplit[j + 1].d0_x
               < potential[k][i] + cliSplit[i + 1].d0_x + sumDistance[j + 1]
                     - sumDistance[i + 1] + MY_EPSILON;
    }

    // Split for unlimited fleet
    int splitSimple(Individual *indiv);

public:
    void generalSplit(Individual *indiv, int nbMaxVehicles);

    // Constructor
    explicit Split(Params *params);
};

#endif
