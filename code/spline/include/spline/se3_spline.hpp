#pragma once

#include <cstdint>
#include <optional>

// TODO(Jack): I wanted to keep these headers hidden, because they are really just an implementation detail of
// Se3Spline. I tried to do this with forward declarations of the two spline classes and using unique pointers here in
// the class, but that did not work because in every translation unit where you want to instantiate a Se3Spline you need
// to have the component spline classes :( This defeats the purpose of the forward declarations as I understood them.
// Maybe this is my lack of understanding, but I think if we really want to achieve a hidden implementation we need to
// do something like PIMPL.
#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

class Se3Spline {
   public:
    Se3Spline(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns);

    void AddControlPoint(Isometry3d const control_point);

    std::optional<Isometry3d> Evaluate(std::uint64_t const t_ns) const;

   private:
    R3SplineState r3_spline_;
    SO3SplineState so3_spline_;
};

}  // namespace reprojection::spline
