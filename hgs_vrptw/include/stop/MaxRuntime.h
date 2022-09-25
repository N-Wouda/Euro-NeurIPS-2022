#ifndef MAXRUNTIME_H
#define MAXRUNTIME_H

#include "StoppingCriterion.h"

#include <chrono>

class MaxRuntime : public StoppingCriterion
{
    using clock = std::chrono::system_clock;
    using seconds = std::chrono::seconds;

    clock::time_point const until;

public:
    bool operator()() override { return clock::now() >= until; }

    /**
     * Sets a maximum run time, in seconds. Note that the counter starts from
     * the moment this object is first constructed.
     */
    explicit MaxRuntime(size_t const maxRuntime)
        : until(clock::now() + seconds(maxRuntime))
    {
    }
};

#endif  // MAXRUNTIME_H
