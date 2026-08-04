
#include "steps/pose_initialization.hpp"

#include "calibration/initialization_methods.hpp"
#include "hashing/hashing.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

PoseInitialization::PoseInitialization(AssetId camera_id, StepId targets_id, StepId camera_info_id,
                                       StepId intrinsics_id, database::CalibrationDatabase& db)
    : camera_id_{camera_id}, targets_id_{targets_id}, targets_{db.ExtractedTargetsSelect(targets_id, camera_id)} {
    // TODO(Jack): Copy and pasted practically verbatim from the intrinsic init. Also copy and pasted almost identically
    // below. Seems like this is a good place for a templated helper function.
    auto const camera_info{db.CameraInfoSelect(camera_info_id, camera_id)};
    if (not camera_info) {
        log->error(
            "{{'camera_info_id': '{}', 'asset_id': '{}', 'msg': 'Attempted to load camera info but result was "
            "empty.'}}",
            camera_info_id.value, camera_id.value);
        std::exit(1);
    }
    camera_info_ = *camera_info;

    auto const intrinsics{db.IntrinsicSelect(intrinsics_id, camera_id)};
    if (not intrinsics) {
        log->error(
            "{{'intrinsics_id': '{}', 'asset_id': '{}', 'msg': 'Attempted to intrinsics but result was empty.'}}",
            intrinsics_id.value, camera_id.value);
        std::exit(1);
    }
    intrinsics_ = *intrinsics;
}

Hash PoseInitialization::CacheKey() const { return hashing::HashArguments(targets_, camera_info_, intrinsics_); }

void PoseInitialization::Execute(StepId step_id, database::CalibrationDatabase& db) const {
    Frames const camera_poses{calibration::PoseInitialization(camera_info_, targets_, intrinsics_)};

    log->info("{{'step_id': {}, 'asset_id': {}, 'num_targets': '{}', 'num_poses: {}}}}}", step_id.value,
              camera_id_.value, std::size(targets_), std::size(camera_poses));

    db.CameraPosesInsert(step_id, targets_id_, camera_id_, camera_poses);

    // TODO(Jack): We need to calculate and insert the reprojection errors!
}

}  // namespace reprojection::steps
