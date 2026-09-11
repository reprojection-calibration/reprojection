
#include "steps/pose_initialization.hpp"

#include "calibration/initialization_methods.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "optimization/bundle_adjustment.hpp"

#include "utilities.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

using Ba = optimization::BundleAdjustment;

PoseInitialization::PoseInitialization(AssetId camera_id, StepId targets_id, StepId camera_info_id, StepId intrinsic_id,
                                       SqlitePtr const db)
    : camera_id_{camera_id},
      targets_id_{targets_id},
      targets_{database::TargetsSelect(db.get(), targets_id, camera_id)},
      camera_info_{ValueOrExit(database::CameraInfoSelect(db.get(), camera_info_id, camera_id), log)},
      intrinsic_{ValueOrExit(database::IntrinsicSelect(db.get(), intrinsic_id, camera_id), log)} {}

Hash PoseInitialization::CacheKey() const { return hashing::HashArguments(targets_, camera_info_, intrinsic_); }

void PoseInitialization::Execute(StepId step_id, SqlitePtr const db) const {
    Frames const camera_poses{calibration::PoseInitialization(camera_info_, targets_, intrinsic_)};

    // TODO(Jack): It is a little crazy how we go from Ba::Problem to Ba::Result to RigState for the output and logging
    // here. But we are currently searching for the stable abstractions and this is where we landed, it will probably
    // change.
    Ba::Problem const problem{Ba::SingleCamProblem(camera_info_, intrinsic_, targets_, camera_poses, {}, camera_id_)};
    auto const rig_state{optimization::ToRigState(Ba::Result(problem))};

    database::RigStateInsert(db.get(), step_id, targets_id_, rig_state);
    auto const errors{optimization::EvaluateResiduals(problem)};
    database::ReprojectionErrorsInsert(db.get(), step_id, targets_id_, errors);

    // TODO(Jack): That we log the problem here is a little confusing as it is not really a problem but a result, but I
    // think it gives all the information the user could possibly want. But I think there exists a better naming or
    // abstraction somewhere out there.
    log->info("{{{}, 'problem': {}}}}}", StepLogInfo{Type(), step_id}, problem);
}

}  // namespace reprojection::steps
