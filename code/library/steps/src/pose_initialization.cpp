
#include "steps/pose_initialization.hpp"

#include "calibration/initialization_methods.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "logging/logging.hpp"
#include "optimization/bundle_adjustment.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

PoseInitialization::PoseInitialization(AssetId camera_id, StepId targets_id, StepId camera_info_id, StepId intrinsic_id,
                                       SqlitePtr const db)
    : camera_id_{camera_id},
      targets_id_{targets_id},
      targets_{database::TargetsSelect(db.get(), targets_id, camera_id)} {
    if (auto const camera_info{database::CameraInfoSelect(db.get(), camera_info_id, camera_id)}) {
        camera_info_ = *camera_info;
    } else {
        log->error("{}", camera_info.error());  // LCOV_EXCL_LINE
        std::exit(1);                           // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    if (auto const intrinsic{database::IntrinsicSelect(db.get(), intrinsic_id, camera_id)}) {
        intrinsic_ = *intrinsic;
    } else {
        log->error("{}", intrinsic.error());  // LCOV_EXCL_LINE
        std::exit(1);                         // LCOV_EXCL_LINE
    }
}

Hash PoseInitialization::CacheKey() const { return hashing::HashArguments(targets_, camera_info_, intrinsic_); }

void PoseInitialization::Execute(StepId step_id, SqlitePtr const db) const {
    Frames const camera_poses{calibration::PoseInitialization(camera_info_, targets_, intrinsic_)};

    log->info("{{'step_id': {}, 'asset_id': {}, 'num_targets': '{}', 'num_poses: {}}}}}", step_id.value,
              camera_id_.value, std::size(targets_), std::size(camera_poses));

    database::CameraPosesInsert(db.get(), step_id, targets_id_, camera_id_, camera_poses);

    // Diagnostic output
    OptimizationState const state{intrinsic_, camera_poses};
    ReprojectionErrors const errors{optimization::ReprojectionError(camera_info_, targets_, state)};
    database::ReprojectionErrorsInsert(db.get(), step_id, targets_id_, camera_id_, errors);
}

}  // namespace reprojection::steps
