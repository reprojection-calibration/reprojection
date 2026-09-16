#include "steps/visual_inertial_opt.hpp"

#include "database/calib_db.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "optimization/visual_inertial_opt.hpp"

#include "utilities.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

VisualInertialOpt::VisualInertialOpt(AssetId const imu_id, StepId const imu_data_id, AssetId const cam_id,
                                     StepId const spline_id, StepId const extrinsic_init_id, StepId const targets_id,
                                     StepId const camera_info_id, StepId const intrinsic_id, int const num_threads,
                                     SqlitePtr const db)
    : imu_id_{imu_id},
      imu_data_id_{imu_data_id},
      imu_data_{database::ImuDataSelect(db.get(), imu_data_id, imu_id)},
      cam_id_{cam_id},
      spline_{std::make_unique<spline::Se3Spline>(
          database::ControlPointsSelect(db.get(), spline_id, cam_id),
          ValueOrExit(database::SplineInfoSelect(db.get(), spline_id, cam_id), log))},
      extrinsic_imu_rig_{ValueOrExit(database::ExtrinsicSelect(db.get(), extrinsic_init_id, imu_id, cam_id), log)},
      gravity_{ValueOrExit(database::GravitySelect(db.get(), extrinsic_init_id), log)},
      targets_id_{targets_id},
      targets_{database::TargetsSelect(db.get(), targets_id, cam_id)},
      camera_info_{ValueOrExit(database::CameraInfoSelect(db.get(), camera_info_id, cam_id), log)},
      intrinsic_{ValueOrExit(database::IntrinsicSelect(db.get(), intrinsic_id, cam_id), log)},
      num_threads_{num_threads} {}

Hash VisualInertialOpt::CacheKey() const {
    return hashing::HashArgs(camera_info_, targets_, intrinsic_, imu_data_, spline_->ControlPoints(),
                             spline_->GetTimeHandler().t0_ns_, spline_->GetTimeHandler().delta_t_ns_,
                             extrinsic_imu_rig_, gravity_);
}

// TODO(Jack): There is really no reason for us to limit us here to only passing the cam0 targets. We should really
// consider this a calibration to the entire stereo rig we optimized before. Assuming we have a stereo rig of course!
void VisualInertialOpt::Execute(StepId step_id, SqlitePtr const db) const {
    // TODO USE SHORTENED NAME SPACE!!!
    // USE ALL CAMERAS! THIS IS HARDCODED TO SINGLE CAM PROBLEM!
    auto const problem{optimization::BundleAdjustment::ViSingleCamProblem(
        camera_info_, intrinsic_, targets_, *spline_, extrinsic_imu_rig_.se3_a_b, gravity_, cam_id_)};

    auto const [result, ceres_state]{optimization::VisualInertialOpt(imu_data_, problem, num_threads_)};

    // TOOD(Jack): Should we add a constructor/factory that just lets us update the se3 part?
    Extrinsic const extrinsic_imu_rig{imu_id_, cam_id_, result.se3_imu_rig};
    log->info("{{{}, 'extrinsic': {}, 'gravity': [{:.3f}], 'solver_summary': {}}}", StepLogInfo{Type(), step_id},
              extrinsic_imu_rig, fmt::join(result.gravity_w, ", "), ceres_state.solver_summary);

    database::SplineInfoInsert(db.get(), step_id, cam_id_, result.rig_spline.GetTimeHandler());
    database::ControlPointsInsert(db.get(), step_id, cam_id_, result.rig_spline.ControlPoints());
    database::GravityInsert(db.get(), step_id, result.gravity_w);

    // Diagnostic output - reprojection errors
    // REFACTOR TO EITHER CALCULTE DIRECRTL FROM VI BA PROBLEM OR CONVERT FROM VI PROBLEM TO REGULAR PROBLEM DIRECTLY!
    auto const ba_problem{
        optimization::SingleSplineCamProblem(camera_info_, intrinsic_, targets_, result.rig_spline, cam_id_)};
    auto const residuals{optimization::EvaluateResiduals(ba_problem)};

    // TODO(Jack): One day if we adopt a spline optimization Result type we can add a transform function to RigState
    // here like we do for the regular bundle adjustment.
    transforms::RigState const rig_state{cam_id_, ba_problem.rig_poses, transforms::Extrinsics{{extrinsic_imu_rig}}};
    database::RigStateInsert(db.get(), step_id, targets_id_, rig_state);
    database::ReprojectionErrorsInsert(db.get(), step_id, {{cam_id_, targets_id_}}, residuals);

    // Diagnostic output - imu errors
    ImuErrors const imu_errors{
        optimization::EvaluateImuError(imu_data_, extrinsic_imu_rig, result.gravity_w, result.rig_spline)};
    database::ImuErrorsInsert(db.get(), step_id, imu_data_id_, imu_id_, imu_errors);
}

}  // namespace reprojection::steps
