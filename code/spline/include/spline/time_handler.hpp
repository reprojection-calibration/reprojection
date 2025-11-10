#pragma once

#include <cstdint>
#include <optional>
#include <tuple>

// https://math.stackexchange.com/questions/2599669/find-control-points-to-produce-a-given-curve

namespace reprojection::spline {

// Calculates what [1] calls "u" - "normalized time elapsed since start of the segment" - see the second paragraph in
// section 4.2 Matrix Representation. In addition to the normalized segment time u we also return the segment index i as
// this is useful information for error/bounds checking and follows the "law of useful return" principle :)
std::tuple<double, int> NormalizedSegmentTime(std::uint64_t const t0_ns, std::uint64_t const t_ns,
                                              std::uint64_t const delta_t_ns);

class TimeHandler {
   public:
    TimeHandler(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns, int const k);

    std::optional<std::tuple<double, int>> SplinePosition(std::uint64_t const t_ns,
                                                          size_t const num_control_points) const;

    std::uint64_t t0_ns_;
    std::uint64_t delta_t_ns_;
    int k_;
};

}  // namespace reprojection::spline