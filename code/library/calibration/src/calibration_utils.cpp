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

std::pair<Frames, Frames> SynchronizeFrames(Frames frames_a, Frames frames_b, uint64_t const sync_delta_ns) {
    Frames frames_b_synced;
    for (auto it_a{std::cbegin(frames_a)}; it_a != std::cend(frames_a);) {
        if (std::empty(frames_b)) {
            // Nothing left in b to sync to so remove all the extra elements from a and break so we can return what
            // could be successfully synced.
            frames_a.erase(it_a, std::cend(frames_a));
            break;
        }

        uint64_t const timestamp_ns_a{it_a->first};
        auto const closest{FindClosest(frames_b, timestamp_ns_a)};

        uint64_t const delta_ns{closest->first > timestamp_ns_a ? closest->first - timestamp_ns_a
                                                                : timestamp_ns_a - closest->first};
        if (delta_ns > sync_delta_ns) {
            // No good match, delete and start next iteration
            it_a = frames_a.erase(it_a);
            continue;
        }

        // Got a good match, copy and frame to the new synchronized map and delete from the old one to enforce
        // one-to-one correspondence.
        frames_b_synced.insert({timestamp_ns_a, closest->second});
        frames_b.erase(closest);

        ++it_a;
    }

    return {frames_a, frames_b_synced};
}

}  // namespace reprojection::calibration
