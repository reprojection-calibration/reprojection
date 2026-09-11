#pragma once

#include <cstdint>
#include <set>

// TODO(Jack): Our current time synchronization logic is extremely simple and unsophisticated. Basically it finds the
// closest element to a timestamp given a set of timestamps, and then checks if it is within the threshold. If so then
// the match is accepted, the element removed from the set, and the process continues. Unfortunately the actually sync
// logic is implemented in multiple places and at this time we only have the methods used to implement it here in this
// central location. It would be important to eliminate code duplication to actually centralize the sync logic itself.
// NOTE(Jack): The sync logic we do makes no guarantees that the best matching across the entire set is made (ex.
// minimizing the sum of time deltas etc.) It is a greedy algorithm and should not be expected to save the world.

namespace reprojection::time_synchronization {

std::set<uint64_t>::const_iterator FindClosest(std::set<uint64_t> const& data, uint64_t timestamp);

bool IsWithinThreshold(uint64_t lhs, uint64_t rhs, uint64_t threshold_ns);

}  // namespace reprojection::time_synchronization
