#include "optimization/bundle_adjustment.hpp"

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

using Ba = optimization::BundleAdjustment;

BundleAdjustment::BundleAdjustment(AssetId const camera_id, StepId const targets_id, int const num_threads,
                                   StepId const camera_info_id, StepId const intrinsic_id, StepId const camera_poses_id,
                                   SqlitePtr const db)
    : camera_id_{camera_id},
      targets_id_{targets_id},
      num_threads_{num_threads},
      targets_{database::TargetsSelect(db.get(), targets_id, camera_id)},
      camera_poses_{database::CameraPosesSelect(db.get(), camera_poses_id, camera_id)} {
    if (auto const camera_info{database::CameraInfoSelect(db.get(), camera_info_id, camera_id)}) {
        camera_info_ = *camera_info;
    } else {
        log->error("{}", camera_info.error());  // LCOV_EXCL_LINE
        std::exit(1);                           // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    if (auto const intrinsics{database::IntrinsicSelect(db.get(), intrinsic_id, camera_id)}) {
        intrinsic_ = *intrinsics;
    } else {
        log->error("{}", intrinsics.error());  // LCOV_EXCL_LINE
        std::exit(1);                          // LCOV_EXCL_LINE
    }
}

Hash BundleAdjustment::CacheKey() const {
    return hashing::HashArguments(camera_info_, targets_, intrinsic_, camera_poses_);
}

void BundleAdjustment::Execute(StepId step_id, SqlitePtr const db) const {
    auto const aligned_camera_poses{calibration::AlignRotations(camera_poses_)};

    Ba::Problem const problem{
        Ba::SingleCamProblem(camera_info_, {intrinsic_}, targets_, aligned_camera_poses, true, camera_id_)};
    auto const [result, ceres_state]{Ba::Solve(problem, num_threads_)};
    auto const& [_, rig_poses, cameras]{result};

    // TODO(Jack): See comment in ba tests about the need for a better asset id independent single camera workflow.
    // NOTE(Jack): The database asset ids start at 1 (sql standard) so an id of zero here is somehow a sentinel value
    // that is unique and "protected".
    auto const intrinsics{cameras.at(camera_id_).intrinsic.value};

    database::RigStateInsert(db.get(), step_id, targets_id_, optimization::ToRigState(result));
    database::IntrinsicInsert(db.get(), step_id, camera_id_, camera_info_.camera_model, {intrinsics});

    // Diagnostic output
    // NOTE(Jack): Here we update the problem with the optimized rig poses and camera states before we evaluate the
    // reprojection error.
    Ba::Problem const optimized_problem{problem, rig_poses, cameras};
    auto const errors{optimization::EvaluateResiduals(optimized_problem)};
    database::ReprojectionErrorsInsert(db.get(), step_id, targets_id_, errors);

    log->info("{{'step_id': {}, 'problem': {}}}}}", step_id.value, problem);
    log->info("{{'step_id': {}, 'result': {}, 'solver_summary': {}}}}}", step_id.value, result,
              ceres_state.solver_summary);
}

}  // namespace reprojection::steps
