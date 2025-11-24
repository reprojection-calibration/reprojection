#include "spline/time_handler.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>  // REMOVE
#include <optional>
#include <tuple>

namespace reprojection::spline {

std::tuple<double, int> NormalizedSegmentTime(std::uint64_t const t0_ns, std::uint64_t const t_ns,
                                              std::uint64_t const delta_t_ns) {
    assert(t0_ns <= t_ns);
    assert(delta_t_ns > 0);

    double const s_t{static_cast<double>(t_ns - t0_ns) / delta_t_ns};
    double const i{std::floor(s_t)};

    return {(s_t - i), static_cast<int>(i)};
}

TimeHandler::TimeHandler(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns, int const k)
    : t0_ns_{t0_ns}, delta_t_ns_{delta_t_ns}, k_{k} {}

std::optional<std::tuple<double, int>> TimeHandler::SplinePosition(std::uint64_t const t_ns,
                                                                   size_t const num_control_points) const {
    auto const [u_i, i]{NormalizedSegmentTime(t0_ns_, t_ns, delta_t_ns_)};

    std::cout << "time: " << t_ns << " u_i: " << u_i << " i: " << i << std::endl;

    // From reference [1] - "At time t in [t_i, t_i+1) the value of p(t) only depends on the control points p_i,
    // p_i+1, ..., p_i+k-1" - See the start of the second paragraph in section 4.2 Matrix Representation.
    if (num_control_points < static_cast<size_t>(i + k_)) {
        return std::nullopt;
    }

    return std::tuple{u_i, i};
}

}  // namespace reprojection::spline