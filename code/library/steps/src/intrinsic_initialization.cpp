#include "steps/intrinsic_initialization.hpp"

#include "calibration/initialization_methods.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"

#include "utilities.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

IntrinsicInitialization::IntrinsicInitialization(AssetId const camera_id, int const num_threads,
                                                 StepId const camera_info_id, StepId const targets_id,
                                                 SqlitePtr const db)
    : camera_id_{camera_id},
      num_threads_{num_threads},
      camera_info_{ValueOrExit(database::CameraInfoSelect(db.get(), camera_info_id, camera_id), log)},
      targets_{database::TargetsSelect(db.get(), targets_id, camera_id)} {}

Hash IntrinsicInitialization::CacheKey() const {
    // NOTE(Jack): See FeatureExtraction::CacheKey() comment as to why we need the camera asset id.
    return hashing::HashArguments(camera_id_.value, camera_info_, targets_);
}

void IntrinsicInitialization::Execute(StepId const step_id, SqlitePtr const db) const {
    auto const intrinsic{calibration::InitializeIntrinsics(camera_info_.camera_model, camera_info_.bounds.v_max,
                                                           camera_info_.bounds.u_max, targets_, num_threads_)};
    if (not intrinsic.has_value()) {
        // LCOV_EXCL_START
        log->error("{{{}, 'msg': 'Failed to initialize intrinsics.'}}", StepLogInfo{Type(), step_id, camera_id_});
        std::exit(1);
        // LCOV_EXCL_STOP
    }

    log->info("{{{}, 'camera_model': '{}', 'intrinsic: {:.3f}}}}}", StepLogInfo{Type(), step_id, camera_id_},
              ToString(camera_info_.camera_model), fmt::join(*intrinsic, ", "));

    database::IntrinsicInsert(db.get(), step_id, camera_id_, camera_info_.camera_model, Intrinsic{*intrinsic});
}

}  // namespace reprojection::steps
