
#include "steps/pose_initialization.hpp"

#include "calibration/initialization_methods.hpp"
#include "hashing/hashing.hpp"
#include "logging/logging.hpp"
#include "optimization/bundle_adjustment.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

PoseInitialization::PoseInitialization(AssetId camera_id, StepId targets_id, StepId camera_info_id,
                                       StepId intrinsics_id, SqlitePtr const db)
    : camera_id_{camera_id},
      targets_id_{targets_id},
      targets_{database::ExtractedTargetsSelect(db.get(), targets_id, camera_id)} {
    // TODO(Jack): Copy and pasted practically verbatim from the intrinsic init. Also copy and pasted almost identically
    // below. Seems like this is a good place for a templated helper function.
    auto const camera_info{database::CameraInfoSelect(db.get(), camera_info_id, camera_id)};
    if (not camera_info) {
        log->error(  // LCOV_EXCL_LINE
            "{{'camera_info_id': '{}', 'asset_id': '{}', 'msg': 'Attempted to load camera info but result was "
            "empty.'}}",
            camera_info_id.value, camera_id.value);
        std::exit(1);  // LCOV_EXCL_LINE
    }
    camera_info_ = *camera_info;

    auto const intrinsics{database::IntrinsicSelect(db.get(), intrinsics_id, camera_id)};
    if (not intrinsics) {
        log->error(  // LCOV_EXCL_LINE
            "{{'intrinsics_id': '{}', 'asset_id': '{}', 'msg': 'Attempted to intrinsics but result was empty.'}}",
            intrinsics_id.value, camera_id.value);
        std::exit(1);  // LCOV_EXCL_LINE
    }
    intrinsics_ = *intrinsics;
}

Hash PoseInitialization::CacheKey() const { return hashing::HashArguments(targets_, camera_info_, intrinsics_); }

void PoseInitialization::Execute(StepId step_id, SqlitePtr const db) const {
    Frames const camera_poses{calibration::PoseInitialization(camera_info_, targets_, intrinsics_)};

    log->info("{{'step_id': {}, 'asset_id': {}, 'num_targets': '{}', 'num_poses: {}}}}}", step_id.value,
              camera_id_.value, std::size(targets_), std::size(camera_poses));

    database::CameraPosesInsert(db.get(), step_id, targets_id_, camera_id_, camera_poses);

    OptimizationState const state{intrinsics_, camera_poses};
    ReprojectionErrors const error{optimization::ReprojectionError(camera_info_, targets_, state)};
    database::ReprojectionErrorsInsert(db.get(), step_id, targets_id_, camera_id_, error);
}

}  // namespace reprojection::steps
