#include <ceres/ceres.h>

#include "spline/so3_spline.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// Exact same as r3 case!
ceres::CostFunction* CreateSo3SplineCostFunction(spline::DerivativeOrder const derivative, Vector3d const& r3,
                                                 double const u_i, std::uint64_t const delta_t_ns);

}  // namespace reprojection::optimization
