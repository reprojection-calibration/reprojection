#pragma once

#include <cstdint>
#include <optional>

#include "spline/spline_state.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

class Se3Spline {
   public:
    Se3Spline(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns);

    Se3Spline(CubicBSplineC3 const& so3_spline, CubicBSplineC3 const& r3_spline);

    void AddControlPoint(Isometry3d const control_point);

    std::optional<Vector6d> Evaluate(std::uint64_t const t_ns) const;

   private:
    CubicBSplineC3 so3_spline_;
    CubicBSplineC3 r3_spline_;
};

}  // namespace reprojection::spline
