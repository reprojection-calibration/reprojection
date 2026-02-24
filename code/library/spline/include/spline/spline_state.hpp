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
 * and the vector of control points.
 */

// TODO(Jack): Reorder so that control points come first!
struct CubicBSplineC3 {
    CubicBSplineC3(TimeHandler const& time_handler,
                   Eigen::Ref<Matrix3Xd const> const& control_points)
        : time_handler_{time_handler}, control_points_{control_points} {}

    // TODO(Jack): Confirm that the mapping actually creates a copy of the data!
    // TODO(Jack): Reorder so that control points come first!
    CubicBSplineC3(TimeHandler const& time_handler, std::vector<Vector3d> const& control_points)
        : CubicBSplineC3(time_handler, Eigen::Map<Matrix3Xd const>(
                                           control_points[0].data(), 3, std::size(control_points))) {}

    Eigen::Ref<Matrix3Xd const> ControlPoints() const { return control_points_; }

    /**
     * \brief The number of control points the spline contains.
     */
    Eigen::Index Size() const { return control_points_.cols(); }

    // TODO MAKE PRIVATE
    TimeHandler time_handler_;

   private:
    Matrix3Xd control_points_;
};

}  // namespace reprojection::spline