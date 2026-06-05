#include "steps/extrinsic_optimization.hpp"

#include "caching/cache_keys.hpp"

namespace reprojection::steps {

std::string ExtrinsicOptimization::CacheKey() const {
    return caching::CacheKey(imu_data, spline.ControlPoints(), spline.GetTimeHandler().t0_ns_,
                             spline.GetTimeHandler().delta_t_ns_, tf_imu_co, gravity_w, camera_info, targets,
                             intrinsics);
}

// TODO(Jack): Use a real defined type?
std::tuple<spline::Se3Spline, Array6d, Array3d> ExtrinsicOptimization::Compute() const {}

std::tuple<spline::Se3Spline, Array6d, Array3d> ExtrinsicOptimization::Load(SqlitePtr const db) const {}

void ExtrinsicOptimization::Save(std::tuple<spline::Se3Spline, Array6d, Array3d> const& state,
                                 SqlitePtr const db) const {}

}  // namespace reprojection::steps
