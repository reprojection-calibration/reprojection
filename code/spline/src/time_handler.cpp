#include "spline/time_handler.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <optional>
#include <tuple>
#include <cassert>

namespace reprojection::spline {

// Calculates what [1] calls "u" - "normalized time elapsed since start of the segment" - see the second paragraph in
// section 4.2 Matrix Representation. In addition to the normalized segment time u we also return the segment index i as
// this is useful information for error/bounds checking and follows the "law of useful return" principle :)
std::tuple<double, int> NormalizedSegmentTime(uint64_t const t0_ns, uint64_t const t_ns, uint64_t const delta_t_ns) {
    assert(t0_ns <= t_ns);
    assert(delta_t_ns > 0);

    double const s_t{static_cast<double>(t_ns - t0_ns) / delta_t_ns};
    double const i{std::floor(s_t)};

    return {(s_t - i), static_cast<int>(i)};
}

TimeHandler::TimeHandler(uint64_t const t0_ns, uint64_t const delta_t_ns, int const k)
    : t0_ns_{t0_ns}, delta_t_ns_{delta_t_ns}, k_{k} {}

std::optional<std::tuple<double, int>> TimeHandler::SplinePosition(uint64_t const t_ns, size_t const num_knots) const {
    auto const [u_i, i]{NormalizedSegmentTime(t0_ns_, t_ns, delta_t_ns_)};

    // From reference [1] - "At time t in [t_i, t_i+1) the value of p(t) only depends on the control points p_i,
    // p_i+1, ..., p_i+k-1" - See the start of the second paragraph in section 4.2 Matrix Representation.
    if (num_knots < static_cast<size_t>(i + k_)) {
        return std::nullopt;
    }

    return std::tuple{u_i, i};
}

}  // namespace reprojection::spline