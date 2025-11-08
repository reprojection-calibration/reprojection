#pragma once

#include <optional>
#include <vector>
#include <cstdint>

#include "spline/time_handler.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// NOTE(Jack): It is a uniform spline which presupposes that all added control_points correspond to specific evenly spaced
// times.
// NOTE(Jack): We static variables in some places because the values are needed in one and only one method, therefore it
// does not make sense to make them part of the class and crowd the class scope.
class r3Spline {
   public:
    r3Spline(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns);

    std::optional<Vector3d> Evaluate(std::uint64_t const t_ns,
                                     DerivativeOrder const derivative = DerivativeOrder::Null) const;

    // TODO(Jack): Let us consider what benefit we would get from making this private at some later point
    std::vector<Vector3d> control_points_;

   private:
    TimeHandler time_handler_;
};

}  // namespace reprojection::spline
