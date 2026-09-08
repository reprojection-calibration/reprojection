
#include "steps/pose_initialization.hpp"

#include "calibration/initialization_methods.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "optimization/bundle_adjustment.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

using Ba = optimization::BundleAdjustment;

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

    // TODO(Jack): It is a little crazy how we go from Ba::Problem to Ba::Result to RigState for the output and logging
    // here. But we are currently searching for the stable abstractions and this is where we landed, it will probably
    // change.
    Ba::Problem const ba_problem{
        Ba::SingleCamProblem(camera_info_, intrinsic_, targets_, camera_poses, {}, camera_id_)};
    auto const rig_state{optimization::ToRigState(Ba::Result(ba_problem))};

    database::RigStateInsert(db.get(), step_id, targets_id_, rig_state);
    auto const errors{optimization::EvaluateResiduals(ba_problem)};
    database::ReprojectionErrorsInsert(db.get(), step_id, targets_id_, errors);

    // TODO(Jack): Should we log the rig state here or the bundle adjustment result? They have some duplicated
    // information but considering pose init uses the intrinsics and targets it almost feels like we should log the BA
    // states here.
    log->info("{{'step_id': {}, 'num_targets': '{}', 'rig_state: {}}}}}", step_id.value, std::size(targets_),
              rig_state);
}

}  // namespace reprojection::steps
