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
    /**
     * \brief Construct a spline time handler given the start time, time increment (time between knots, constant for
     * uniform splines used here), and the spline order.
     *
     * @param t0_ns Start time in nanoseconds
     * @param delta_t_ns Time increment between knots in nanoseconds. For uniform splines all knots are incremented
     * equally, therefore if a knot is added we can also say that the spline is delta_t_ns longer.
     * @param k Spline order. Evaluating a spline at time t in range [t_i, t_i+1) requires k-1 knots
     */
    TimeHandler(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns, int const k);

    std::optional<std::tuple<double, int>> SplinePosition(std::uint64_t const t_ns, size_t const num_knots) const;

    std::uint64_t t0_ns_;
    std::uint64_t delta_t_ns_;
    int k_;
};

}  // namespace reprojection::spline