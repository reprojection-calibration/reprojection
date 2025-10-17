#pragma once

#include <optional>

#include "types.hpp"
#include "utilities.hpp"

namespace reprojection::spline {

// NOTE(Jack): It is a uniform spline which presupposes that all added knots correspond to specific evenly spaced
// times.
// NOTE(Jack): We static variables in some places because the values are needed in one and only one method, therefore it
// does not make sense to make them part of the class and crowd the class scope.
class r3Spline {
   public:
    r3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns);

    std::optional<VectorD> Evaluate(uint64_t const t_ns,
                                    DerivativeOrder const derivative = DerivativeOrder::Null) const;

    // TODO(Jack): Factor this out as we use the same method for the SO3 spline as well!
    static VectorK CalculateU(double const u_i, DerivativeOrder const derivative = DerivativeOrder::Null);

    // TODO(Jack): Let us consider what benefit we would get from making this private at some later point
    std::vector<VectorD> knots_;  // A.k.a. "control points"

   private:
    TimeHandler time_handler_;
};

}  // namespace reprojection::spline
