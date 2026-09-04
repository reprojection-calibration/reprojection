#include "optimization/bundle_adjustment.hpp"

#include <ranges>

#include "calibration/calibration_utils.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "steps/bundle_adjustment.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

BundleAdjustment::BundleAdjustment(AssetId const camera_id, StepId const targets_id, int const num_threads,
                                   StepId const camera_info_id, StepId const intrinsic_id, StepId const camera_poses_id,
                                   SqlitePtr const db)
    : camera_id_{camera_id},
      targets_id_{targets_id},
      num_threads_{num_threads},
      targets_{database::ExtractedTargetsSelect(db.get(), targets_id, camera_id)},
      camera_poses_{database::CameraPosesSelect(db.get(), camera_poses_id, camera_id)} {
    if (auto const camera_info{database::CameraInfoSelect(db.get(), camera_info_id, camera_id)}) {
        camera_info_ = *camera_info;
    } else {
        log->error("{}", camera_info.error());  // LCOV_EXCL_LINE
        std::exit(1);                           // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    if (auto const intrinsics{database::IntrinsicSelect(db.get(), intrinsic_id, camera_id)}) {
        intrinsics_ = *intrinsics;
    } else {
        log->error("{}", intrinsics.error());  // LCOV_EXCL_LINE
        std::exit(1);                          // LCOV_EXCL_LINE
    }
}

Hash BundleAdjustment::CacheKey() const {
    return hashing::HashArguments(camera_info_, targets_, intrinsics_, camera_poses_);
}

void BundleAdjustment::Execute(StepId step_id, SqlitePtr const db) const {
    auto const aligned_camera_poses{calibration::AlignRotations(camera_poses_)};

    optimization::BundleAdjustment::Problem const problem{
        optimization::BuildSingleCamBaProblem(camera_info_, {intrinsics_}, aligned_camera_poses, targets_)};
    auto const [frames, ceres_state, cameras]{optimization::BundleAdjust(problem, num_threads_)};

    // TODO(Jack): See comment in ba tests about the need for a better asset id independent single camera workflow.
    // NOTE(Jack): The database asset ids start at 1 (sql standard) so an id of zero here is somehow a sentinel value
    // that is unique and "protected".
    auto const intrinsics{cameras.at(AssetId{0}).intrinsic.value};

    log->info(
        "{{'step_id': {}, 'asset_id': {}, 'camera_model': '{}', 'intrinsic: {}, 'solver_summary': {{'intial_cost': "
        "{:.2f}, 'final_cost': {:.2f}, 'num_successful_steps': {}, 'num_unsuccessful_steps': {}}}}}}}",
        step_id.value, camera_id_.value, ToString(camera_info_.camera_model), intrinsics,
        ceres_state.solver_summary.initial_cost, ceres_state.solver_summary.final_cost,
        ceres_state.solver_summary.num_successful_steps, ceres_state.solver_summary.num_unsuccessful_steps);

    database::CameraPosesInsert(db.get(), step_id, targets_id_, camera_id_, frames);
    database::IntrinsicInsert(db.get(), step_id, camera_id_, camera_info_.camera_model, {intrinsics});

    // Diagnostic output
    ReprojectionErrors const errors{optimization::ReprojectionError(camera_info_, targets_, {{intrinsics}, frames})};
    database::ReprojectionErrorsInsert(db.get(), step_id, targets_id_, camera_id_, errors);
}

}  // namespace reprojection::steps
