
#include "time_synchronization.hpp"

#include <stdexcept>

namespace reprojection::calibration {

std::set<uint64_t>::const_iterator FindClosest(std::set<uint64_t> const& data, uint64_t const timestamp) {
    // TODO(Jack): I highly doubt throwing at this point is the right strategy here, but for now lets fail loud!
    if (std::empty(data)) {
        throw std::runtime_error("FindClosest(): data is empty!");
    }

    auto const upper{data.lower_bound(timestamp)};
    if (upper == std::cbegin(data)) {
        return upper;
    } else if (upper == std::cend(data)) {
        return std::prev(upper);
    }

    auto const lower{std::prev(upper)};

    uint64_t const upper_delta{*upper - timestamp};
    uint64_t const lower_delta{timestamp - *lower};

    return lower_delta <= upper_delta ? lower : upper;
}

bool IsWithinThreshold(uint64_t const lhs, uint64_t const rhs, uint64_t const threshold_ns) {
    // NOTE(Jack): We need this kinda funky looking logic because we are dealing with unsigned types and need to worry
    // about NOT creating negative numbers that will underflow. Probably was a dumb idea to use an unsigned type in the
    // first place.
    uint64_t const delta{lhs > rhs ? lhs - rhs : rhs - lhs};

    return delta <= threshold_ns;
}

}  // namespace reprojection::calibration
