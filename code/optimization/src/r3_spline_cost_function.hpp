#include <ceres/ceres.h>

#include "spline/r3_spline.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

ceres::CostFunction* CreateR3SplineCostFunction(spline::DerivativeOrder const derivative, Vector3d const& r3,
                                                double const u_i, std::uint64_t const delta_t_ns);


}  // namespace reprojection::optimization
