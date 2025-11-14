#pragma once

#include <cstdint>
#include <optional>

#include "spline/spline_state.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

class Se3Spline {
   public:
    Se3Spline(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns);

    void AddControlPoint(Isometry3d const control_point);

    std::optional<Isometry3d> Evaluate(std::uint64_t const t_ns) const;

   private:
    CubicBSplineC3 r3_spline_;
    CubicBSplineC3 so3_spline_;
};

}  // namespace reprojection::spline
