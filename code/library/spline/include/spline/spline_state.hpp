#pragma once

#include <cstdint>
#include <vector>

#include "spline/constants.hpp"
#include "spline/time_handler.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

/**
 * \brief Container for the state of a cubic b-spline that has control points of size 3x1.
 *
 * The name "cubic b-spline c3" means that it is a cubic b-spline with three-dimensional control points. This works
 * conveniently for both Euclidean R3 splines and so3 rotations spline stored in the axis angle representation. The
 * state of the spline consists of a time handler initialized at construction with the starting time and time increment
 * and the vector of control points. Both the time handler and control points are public and meant to be used directly
 * by the user.
 */
struct CubicBSplineC3 {
    CubicBSplineC3(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns)
        : time_handler{t0_ns, delta_t_ns, constants::order} {}

    /**
     * \brief The number of control points the spline contains.
     */
    size_t Size() const { return std::size(control_points); }

    TimeHandler time_handler;
    std::vector<Vector3d> control_points;
};

}  // namespace reprojection::spline