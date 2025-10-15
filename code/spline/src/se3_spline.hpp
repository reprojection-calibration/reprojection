#pragma once

#include <Eigen/Geometry>
#include <optional>

#include "r3_spline.hpp"
#include "so3_spline.hpp"

namespace reprojection::spline {

class Se3Spline {
   public:
    Se3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns);

    void AddKnot(Eigen::Isometry3d const knot);

    std::optional<Eigen::Isometry3d> Evaluate(uint64_t const t_ns) const;

   private:
    r3Spline r3_spline_;
    So3Spline so3_spline_;
};

}  // namespace reprojection::spline
