#pragma once

#include "types.hpp"
#include "utilities.hpp"

namespace reprojection::spline {

// TODO(Jack): There is a non-trivial amount of copying and pasting between the r3 and s03 spline classes. Keep our eyes
// peeled for cost effective and well abstracted optimizations!
class So3Spline {
   public:
    So3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns);

    std::optional<Eigen::Matrix3d> Evaluate(uint64_t const t_ns) const;

    std::optional<Eigen::Vector3d> EvaluateVelocity(uint64_t const t_ns) const;

    std::optional<Eigen::Vector3d> EvaluateAcceleration(uint64_t const t_ns) const;

    // NOTE(Jack): It would feel more natural to store the so3 vectors here but the math required in the evaluate
    // function happens more in the SO3 space so it makes more sense to have the knots be in that format - it is also
    // what people would expect to get returned from the Evaluate() function, so we are consistent.
    // TODO(Jack): When adding a knot should we check that it is a rotation matrix?
    std::vector<Eigen::Matrix3d> knots_;

   private:
    TimeHandler time_handler_;
    MatrixKK const M_;
};

}  // namespace reprojection::spline
