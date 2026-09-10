#pragma once

#include <cstdint>
#include <set>

namespace reprojection::calibration {

std::set<uint64_t>::const_iterator FindClosest(std::set<uint64_t> const& data, uint64_t timestamp);

bool IsWithinThreshold(uint64_t lhs, uint64_t rhs, uint64_t threshold_ns);

}  // namespace reprojection::calibration
