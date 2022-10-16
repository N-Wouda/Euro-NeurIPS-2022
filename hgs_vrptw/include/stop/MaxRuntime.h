#ifndef MAXRUNTIME_H
#define MAXRUNTIME_H

#include "StoppingCriterion.h"

#include <chrono>

class MaxRuntime : public StoppingCriterion
{
    using clock = std::chrono::system_clock;
    using fsec = std::chrono::duration<double>;

    double maxRuntime;
    clock::time_point const start;

public:
    bool operator()() override
    {
        return fsec(clock::now() - start).count() >= maxRuntime;
    }

    /**
     * Sets a maximum run time, in seconds. Note that the counter starts from
     * the moment this object is first constructed.
     */
    explicit MaxRuntime(double const maxRuntime)
        : maxRuntime(maxRuntime), start(clock::now())
    {
        if (maxRuntime <= 0)
            throw std::runtime_error("Run-time of <0s is not understood.");
    }
};

#endif  // MAXRUNTIME_H
