#include "optimization/bundle_adjustment.hpp"

#include <ranges>

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
                                   database::CalibrationDatabase& db)
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

void BundleAdjustment::Execute(StepId step_id, database::CalibrationDatabase& db) const {
    auto const aligned_initial_state{AlignRotations({intrinsics_, camera_poses_})};

    auto const [optimized_state,
                debug]{optimization::BundleAdjustment(camera_info_, targets_, aligned_initial_state, num_threads_)};

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

// WARN(Jack): This is a hack that we need to do so that the spline initialization does not have any massive
// discontinuities or sudden jumps. But there is some bigger problem here that we are missing and need to solve long
// term.
// WARN(Jack): Also note that we do not save aligned_initial_state to the database, we save plain old initial_state and
// use that to calculate the reprojection errors, but use aligned_initial_state to initialize the nonlinear
// optimization. This means that what we are doing here and what we are visualizing in the database are starting to
// diverge. Not nice!
// cppcheck-suppress passedByValue
OptimizationState AlignRotations(OptimizationState state) {
    if (std::empty(state.frames)) {
        return state;
    }

    Vector3d so3_i_1{std::cbegin(state.frames)->second.pose.head<3>()};
    for (auto& frame_i : state.frames | std::views::values) {
        Vector3d so3_i{frame_i.pose.head<3>()};
        double const dp{so3_i_1.dot(so3_i)};

        if (dp < 0) {
            so3_i *= -1.0;
        }
        frame_i.pose.head<3>() = so3_i;

        so3_i_1 = so3_i;
    }

    return state;
}

}  // namespace reprojection::steps
