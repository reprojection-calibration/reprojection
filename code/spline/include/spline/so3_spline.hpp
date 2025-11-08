#pragma once

#include "spline/time_handler.hpp"
#include "types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

class So3Spline {
   public:
    So3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns);

    std::optional<Matrix3d> Evaluate(uint64_t const t_ns) const;

    std::optional<Vector3d> EvaluateVelocity(uint64_t const t_ns) const;

    std::optional<Vector3d> EvaluateAcceleration(uint64_t const t_ns) const;

    // NOTE(Jack): It would feel more natural to store the so3 vectors here but the math required in the evaluate
    // function happens more in the SO3 space so it makes more sense to have the control_points be in that format - it
    // is also what people would expect to get returned from the Evaluate() function, so we are consistent.
    // TODO(Jack): When adding a control_point should we check that it is a rotation matrix?
    std::vector<Matrix3d> control_points_;

   private:
    TimeHandler time_handler_;
    MatrixKK const M_;
};

}  // namespace reprojection::spline
