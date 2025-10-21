#pragma once

#include <optional>

#include "spline/time_handler.hpp"
#include "spline/types.hpp"

namespace reprojection::spline {

// NOTE(Jack): It is a uniform spline which presupposes that all added knots correspond to specific evenly spaced
// times.
// NOTE(Jack): We static variables in some places because the values are needed in one and only one method, therefore it
// does not make sense to make them part of the class and crowd the class scope.
class r3Spline {
   public:
    r3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns);

    std::optional<Eigen::Vector3d> Evaluate(uint64_t const t_ns,
                                            DerivativeOrder const derivative = DerivativeOrder::Null) const;

    // TODO(Jack): Let us consider what benefit we would get from making this private at some later point
    std::vector<Eigen::Vector3d> knots_;  // A.k.a. "control points"

   private:
    TimeHandler time_handler_;
};

}  // namespace reprojection::spline
