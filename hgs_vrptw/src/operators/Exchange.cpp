#include "Exchange.h"

#include "Route.h"
#include "TimeWindowSegment.h"

namespace
{
using TWS = TimeWindowSegment;
}

template <size_t N, size_t M> bool Exchange<N, M>::test(Node *U, Node *V)
{
    // TODO
    return false;
}

template <size_t N, size_t M> void Exchange<N, M>::apply(Node *U, Node *V)
{
    // TODO
}

// Explicit instantiations of the few moves we *might* want to have
template class Exchange<1, 0>;
template class Exchange<2, 0>;
template class Exchange<3, 0>;
template class Exchange<1, 1>;
template class Exchange<2, 1>;
template class Exchange<3, 1>;
template class Exchange<2, 2>;
template class Exchange<3, 2>;
template class Exchange<3, 3>;
