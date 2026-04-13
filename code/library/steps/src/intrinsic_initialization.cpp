#include "steps/intrinsic_initialization.hpp"

#include "caching/cache_keys.hpp"
#include "calibration/initialization_methods.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"

namespace reprojection::steps {

std::string IntrinsicInitializationStep::CacheKey() const { return caching::CacheKey(camera_info, targets); }

CameraState IntrinsicInitializationStep::Compute() const {
    // TODO(Jack): Confirm v and u are height and width in the correct order!
    auto const intrinsics{calibration::InitializeIntrinsics(camera_info.camera_model, camera_info.bounds.v_max,
                                                            camera_info.bounds.u_max, targets)};

    if (not intrinsics.has_value()) {
        throw std::runtime_error("We have no error handling strategy for failed IiStep::Compute()");  // LCOV_EXCL_LINE
    }

    return CameraState{*intrinsics};
}

CameraState IntrinsicInitializationStep::Load(SqlitePtr const& db) const {
    auto const loaded_intrinsics{
        database::ReadCameraState(db, step_type, camera_info.sensor_name, camera_info.camera_model)};

    if (not loaded_intrinsics.has_value()) {
        throw std::runtime_error("We have no error handling strategy for failed IiStep::Load()");  // LCOV_EXCL_LINE
    }

    return CameraState{*loaded_intrinsics};
}

void IntrinsicInitializationStep::Save(CameraState const& intrinsics, SqlitePtr const& db) const {
    database::WriteToDb(intrinsics, camera_info.camera_model, step_type, camera_info.sensor_name, db);
}

}  // namespace reprojection::steps
