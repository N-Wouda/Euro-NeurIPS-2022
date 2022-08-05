#include "Node.h"

#include <cassert>

TimeWindowSegment Node::mergeSegmentTwData(Node const *other) const
{
    assert(route == other->route);
    assert(position <= other->position);

    if (isDepot)
        return other->twBefore;

    if (other->isDepot)
        return twAfter;

    Node const *node = this;
    TimeWindowSegment data = tw;

    while (node != other)
    {
        if (node->isSeed && node->position + 4 <= other->position)
        {
            data = TimeWindowSegment::merge(data, node->toNextSeedTwD);
            node = node->nextSeed;
        }
        else
        {
            node = node->next;
            data = TimeWindowSegment::merge(data, node->tw);
        }
    }

    return data;
}
