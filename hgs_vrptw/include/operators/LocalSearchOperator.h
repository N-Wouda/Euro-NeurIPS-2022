#ifndef LOCALSEARCHOPERATOR_H
#define LOCALSEARCHOPERATOR_H

#include "Individual.h"
#include "Penalties.h"

template <typename Arg> class LocalSearchOperator
{
protected:
    Penalties const *d_penalties = nullptr;

public:
    /**
     * Called once after loading in the solution to improve. This can be used
     * to e.g. update local operator state.
     */
    virtual void init(Individual const &indiv, Penalties const *penalties)
    {
        d_penalties = penalties;
    };

    /**
     * Tests whether applying this operator to the given arguments constitutes
     * an improving move. Returns true if that is the case, false otherwise.
     * Default false (no-op).
     */
    virtual bool test(Arg *U, Arg *V) { return false; }

    /**
     * Applies this operator to the given arguments. For improvements, should
     * only be called if <code>test()</code> returns true. Default no-op.
     */
    virtual void apply(Arg *U, Arg *V){};

    virtual ~LocalSearchOperator() = default;
};

#endif  // LOCALSEARCHOPERATOR_H
