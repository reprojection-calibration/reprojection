#include "steps/intrinsic_initialization.hpp"

#include "calibration/initialization_methods.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

IntrinsicInitialization::IntrinsicInitialization(AssetId const camera_id, int const num_threads,
                                                 StepId const camera_info_id, StepId const targets_id,
                                                 SqlitePtr const db)
    : camera_id_{camera_id}, num_threads_{num_threads} {
    if (auto const camera_info{database::CameraInfoSelect(db.get(), camera_info_id, camera_id)}) {
        camera_info_ = *camera_info;
    } else {
        log->error("{}", camera_info.error());  // LCOV_EXCL_LINE
        std::exit(1);                           // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    targets_ = database::ExtractedTargetsSelect(db.get(), targets_id, camera_id);
}

Hash IntrinsicInitialization::CacheKey() const { return hashing::HashArguments(camera_info_, targets_); }

void IntrinsicInitialization::Execute(StepId const step_id, SqlitePtr const db) const {
    auto const intrinsics{calibration::InitializeIntrinsics(camera_info_.camera_model, camera_info_.bounds.v_max,
                                                            camera_info_.bounds.u_max, targets_, num_threads_)};
    if (not intrinsics.has_value()) {
        log->error("{{'step_id': {}, 'asset_id': {}, 'msg': 'Failed to initialize intrinsics.'}}",  // LCOV_EXCL_LINE
                   step_id.value, camera_id_.value);                                                // LCOV_EXCL_LINE
        std::exit(1);                                                                               // LCOV_EXCL_LINE
    }

    log->info("{{'step_id': {}, 'asset_id': {}, 'camera_model': '{}', 'intrinsic: {}}}}}", step_id.value,
              camera_id_.value, ToString(camera_info_.camera_model), *intrinsics);

    database::IntrinsicInsert(db.get(), step_id, camera_id_, camera_info_.camera_model, CameraState{*intrinsics});
}

}  // namespace reprojection::steps
