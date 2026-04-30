#pragma once

#include <memory>
#include <peak/peak.hpp>

/**
 * @brief Small helper for lazy GenICam node lookup.
 *
 * IDS_CCD keeps the actual node pointers because the existing code already uses
 * those members in many places. This helper centralizes the repeated pattern:
 * find a node only once, then reuse the cached shared_ptr.
 */
class IDSNodeCache
{
public:
    template <typename NodeT>
    std::shared_ptr<NodeT> get(std::shared_ptr<NodeT> &cache,
                               const std::shared_ptr<peak::core::NodeMap> &nodeMap,
                               const char *name) const
    {
        if (!cache && nodeMap)
            cache = nodeMap->FindNode<NodeT>(name);

        return cache;
    }
};
