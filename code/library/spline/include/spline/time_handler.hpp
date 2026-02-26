#pragma once

#include <cstdint>
#include <optional>
#include <tuple>

// https://math.stackexchange.com/questions/2599669/find-control-points-to-produce-a-given-curve

namespace reprojection::spline {

class TimeHandler {
   public:
    TimeHandler(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns);

    TimeHandler();

    // NOTE(Jack): We do not really need this, but I am paranoid and there is one piece of the code which scares me, and
    // that is the InitializeSe3SplineState() function. In that function we call the InitializeC3SplineState() function
    // twice, which means we get two TimeHandler objects. Of course, they should be the same always, so we could
    // theoretically just ignore one and return one as the time handler for the entire se3 spline. However, like I said,
    // I am paranoid, so I need to assert that they are the same, therefore we need to add this comparison operator :)
    bool operator==(TimeHandler const&) const = default;

    std::optional<std::pair<double, int>> SplinePosition(std::uint64_t const t_ns,
                                                         size_t const num_control_points) const;

    // Calculates what [1] calls "u" - "normalized time elapsed since start of the segment" - see the second paragraph
    // in section 4.2 Matrix Representation. In addition to the normalized segment time u we also return the segment
    // index i as this is useful information for error/bounds checking and follows the "law of useful return" principle
    // :)
    static std::pair<double, int> NormalizedSegmentTime(std::uint64_t const t0_ns, std::uint64_t const t_ns,
                                                        std::uint64_t const delta_t_ns);

    std::uint64_t t0_ns_;
    std::uint64_t delta_t_ns_;
};

}  // namespace reprojection::spline