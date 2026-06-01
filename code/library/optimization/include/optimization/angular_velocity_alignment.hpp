#pragma once

#include <tuple>

#include "spline/spline_state.hpp"
#include "types/ceres_types.hpp"
#include "types/eigen_types.hpp"
#include "types/spline_types.hpp"

namespace reprojection::optimization {

// UPDATE DOCS
// UPDATE DOCS
// UPDATE DOCS
// UPDATE DOCS
// UPDATE DOCS
// UPDATE DOCS
// UPDATE DOCS
/**
 * \brief Given two correspondent sets of angular velocities optimize the rotation matrix which minimizes their element
 * wise difference (i.e. minimize {omega_a - R_a_b * omega_b} as a function of R_a_b).
 *
 * If there is no angular velocity, or not all three axes are sufficiently excited, then the problem will be
 * underconstrained and produce a degenerate result.
 */
std::pair<Array3d, CeresState> AngularVelocityAlignment(VelocityMeasurements const& omega_imu,
                                                        spline::CubicBSplineC3 const& so3_spline);

}  // namespace  reprojection::optimization
