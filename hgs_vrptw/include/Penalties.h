#ifndef PENALTIES_H
#define PENALTIES_H

#include "Params.h"
#include "TimeWindowSegment.h"

class Penalties
{
    Params const *params;
    int loadPenalty;
    int timePenalty;

public:
    // Computes the total excess capacity penalty for the given load
    [[nodiscard]] inline int load(int currLoad) const
    {
        auto const excessLoad = currLoad - params->vehicleCapacity;
        return std::max(excessLoad, 0) * loadPenalty;
    }

    // Computes the total time warp penalty for the given time window data
    [[nodiscard]] inline int timeWarp(TimeWindowSegment const &twData) const
    {
        return twData.totalTimeWarp() * timePenalty;
    }

    Penalties(Params const *params, int loadPenalty, int timePenalty)
        : params(params), loadPenalty(loadPenalty), timePenalty(timePenalty)
    {
    }
};

#endif  // PENALTIES_H
