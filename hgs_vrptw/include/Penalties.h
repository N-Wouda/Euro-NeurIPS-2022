#ifndef PENALTIES_H
#define PENALTIES_H

#include "TimeWindowSegment.h"

class Penalties
{
    int vehicleCapacity;
    int loadPenalty;
    int timePenalty;

public:
    // Computes the total excess capacity penalty for the given load
    [[nodiscard]] int load(int currLoad) const
    {
        return std::max(currLoad - vehicleCapacity, 0) * loadPenalty;
    }

    // Computes the total time warp penalty for the given time window data
    [[nodiscard]] int timeWarp(TimeWindowSegment const &twData) const
    {
        return twData.totalTimeWarp() * timePenalty;
    }

    Penalties(int vehicleCapacity, int loadPenalty, int timePenalty)
        : vehicleCapacity(vehicleCapacity),
          loadPenalty(loadPenalty),
          timePenalty(timePenalty)
    {
    }
};

#endif  // PENALTIES_H
