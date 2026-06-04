#include "steps/spline_reprojection_error.hpp"

#include "caching/cache_keys.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "optimization/camera_imu_calibration.hpp"

namespace reprojection::steps {

std::string SplineReprojectionError::CacheKey() const {
    return caching::CacheKey(camera_info, targets, intrinsics, spline.ControlPoints(), spline.GetTimeHandler().t0_ns_,
                             spline.GetTimeHandler().delta_t_ns_);
}

DoNotUse SplineReprojectionError::Compute() const { return DoNotUse{}; }

DoNotUse SplineReprojectionError::Load(SqlitePtr const db) const {
    static_cast<void>(db);

    return DoNotUse{};
}

void SplineReprojectionError::Save(DoNotUse const do_not_use, SqlitePtr const db) const {
    static_cast<void>(do_not_use);
    static_cast<void>(db);  // REMOVE


}

}  // namespace reprojection::steps
