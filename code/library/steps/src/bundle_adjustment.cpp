#include "optimization/bundle_adjustment.hpp"

#include <ranges>

#include "calibration/calibration_utils.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "steps/bundle_adjustment.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

BundleAdjustment::BundleAdjustment(AssetId const camera_id, StepId targets_id, int const num_threads,
                                   StepId const camera_info_id, StepId const intrinsic_id, StepId const camera_poses_id,
                                   database::CalibrationDatabase const& db)
    : camera_id_{camera_id},
      targets_id_{targets_id},
      num_threads_{num_threads},
      targets_{db.ExtractedTargetsSelect(targets_id, camera_id)},
      camera_poses_{db.CameraPosesSelect(camera_poses_id, camera_id)} {
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

    auto const intrinsics{db.IntrinsicSelect(intrinsic_id, camera_id)};
    if (not intrinsics) {
        log->error("{{'intrinsic_id': '{}', 'asset_id': '{}', 'msg': 'Attempted to intrinsics but result was empty.'}}",
                   intrinsic_id.value, camera_id.value);
        std::exit(1);
    }
    intrinsics_ = *intrinsics;
}

Hash BundleAdjustment::CacheKey() const {
    return hashing::HashArguments(camera_info_, targets_, intrinsics_, camera_poses_);
}

void BundleAdjustment::Execute(StepId step_id, database::CalibrationDatabase const& db) const {
    auto const aligned_camera_poses{calibration::AlignRotations(camera_poses_)};
    OptimizationState const initial_state{intrinsics_, aligned_camera_poses};

    auto const [optimized_state,
                debug]{optimization::BundleAdjustment(camera_info_, targets_, initial_state, num_threads_)};

    log->info(
        "{{'step_id': {}, 'asset_id': {}, 'camera_model': '{}', 'intrinsic: {}, 'solver_summary': {{'intial_cost': "
        "{:.2f}, 'final_cost': {:.2f}, 'num_successful_steps': {}, 'num_unsuccessful_steps': {}}}}}}}",
        step_id.value, camera_id_.value, ToString(camera_info_.camera_model), optimized_state.camera_state.intrinsics,
        debug.solver_summary.initial_cost, debug.solver_summary.final_cost, debug.solver_summary.num_successful_steps,
        debug.solver_summary.num_unsuccessful_steps);

    db.CameraPosesInsert(step_id, targets_id_, camera_id_, optimized_state.frames);
    db.IntrinsicInsert(step_id, camera_id_, camera_info_.camera_model, optimized_state.camera_state);

    // TODO(Jack): We need to calculate and insert the reprojection errors!
}

}  // namespace reprojection::steps
