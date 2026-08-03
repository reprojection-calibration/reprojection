#include "steps/intrinsic_initialization.hpp"

#include "calibration/initialization_methods.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

IntrinsicInitialization::IntrinsicInitialization(AssetId const camera_id, int const num_threads,
                                                 StepId const camera_info_id, StepId const targets_id,
                                                 database::CalibrationDatabase& db)
    : camera_id_{camera_id}, num_threads_{num_threads} {
    auto const camera_info{db.CameraInfoSelect(camera_info_id, camera_id)};
    if (not camera_info) {
        // TODO LOG ERROR!
        std::terminate();
    }
    camera_info_ = *camera_info;

    targets_ = db.ExtractedTargetsSelect(targets_id, camera_id);
}

Hash IntrinsicInitialization::CacheKey() const { return hashing::HashArguments(camera_info_, targets_); }

void IntrinsicInitialization::Execute(StepId const step_id, database::CalibrationDatabase& db) const {
    auto const intrinsics{calibration::InitializeIntrinsics(camera_info_.camera_model, camera_info_.bounds.v_max,
                                                            camera_info_.bounds.u_max, targets_, num_threads_)};

    // TODO UNIFY ERROR HANDLING LOGGING!
    if (not intrinsics.has_value()) {
        throw std::runtime_error(
            "We have no error handling strategy for failed IntrinsicInitializationStep::Compute()");
    }

    log->info("{{'step_id': '{}', 'asset_id': '{}', 'camera_model': '{}', 'intrinsic: {}}}}}", step_id.value,
              camera_id_.value, ToString(camera_info_.camera_model), *intrinsics);

    db.IntrinsicInsert(step_id, camera_id_, camera_info_.camera_model, CameraState{*intrinsics});
}

}  // namespace reprojection::steps
