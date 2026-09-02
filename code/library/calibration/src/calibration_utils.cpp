#include "calibration/calibration_utils.hpp"

#include <ranges>

namespace reprojection::calibration {

// WARN(Jack): This is a hack that we need to do so that the spline initialization does not have any massive
// discontinuities or sudden jumps. But there is some bigger problem here that we are missing and need to solve long
// term.
Frames AlignRotations(Frames data) {
    if (std::empty(data)) {
        return data;
    }

    Vector3d so3_i_1{std::cbegin(data)->second.pose.head<3>()};
    for (auto& frame_i : data | std::views::values) {
        Vector3d so3_i{frame_i.pose.head<3>()};
        double const dp{so3_i_1.dot(so3_i)};

        if (dp < 0) {
            so3_i *= -1.0;
        }
        frame_i.pose.head<3>() = so3_i;

        so3_i_1 = so3_i;
    }

    return data;
}

// TODO TEST? and split between hpp and cpp?
auto FindClosest(Frames const& data, uint64_t const timestamp) {
    auto const upper{data.lower_bound(timestamp)};
    if (upper == std::cbegin(data)) {
        return upper;
    } else if (upper == std::cend(data)) {
        return std::prev(upper);
    }

    auto const lower{std::prev(upper)};

    uint64_t const upper_delta{upper->first - timestamp};
    uint64_t const lower_delta{timestamp - lower->first};

    return lower_delta <= upper_delta ? lower : upper;
}

// NOTE(Jack): We pass frames_a by value because we edit it and remove unsyncable entries and we pass frames_b by const
// reference because we only copy out the values we actually need from it.
std::pair<Frames, Frames> SynchronizeFrames(Frames frames_a, Frames const& frames_b, uint64_t const sync_delta_ns) {
    Frames frames_b_synced;
    for (auto const& [timestamp_ns_a, frame_a] : frames_a) {
        auto const closest{FindClosest(frames_b, timestamp_ns_a)};

        // NOTE(Jack): We used the unsigned long type so we need to be careful to protect against underflow here! Not
        // 100% sure this is right but I think it works for us.
        uint64_t const delta_ns{closest->first > timestamp_ns_a ? closest->first - timestamp_ns_a
                                                                : timestamp_ns_a - closest->first};
        if (delta_ns > sync_delta_ns) {
            frames_a.erase(timestamp_ns_a);
            continue;
        }

        frames_b_synced.insert({timestamp_ns_a, closest->second});
    }

    return {frames_a, frames_b_synced};
}

}  // namespace reprojection::calibration
