#include "optimization/extrinsic_optimization.hpp"

#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "steps/extrinsic_optimization.hpp"

#include "utilities.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

VisualInertialOpt::VisualInertialOpt(AssetId const camera_id, AssetId const imu_id, StepId const targets_id,
                                             StepId const imu_data_id, int const num_threads,
                                             StepId const camera_info_id, StepId const intrinsic_id,
                                             StepId const spline_id, StepId const extrinsic_init_id, SqlitePtr const db)
    : camera_id_{camera_id},
      imu_id_{imu_id},
      targets_id_{targets_id},
      targets_{database::TargetsSelect(db.get(), targets_id, camera_id)},
      imu_data_id_{imu_data_id},
      imu_data_{database::ImuDataSelect(db.get(), imu_data_id, imu_id)},
      num_threads_{num_threads},
      camera_info_{ValueOrExit(database::CameraInfoSelect(db.get(), camera_info_id, camera_id), log)},
      intrinsic_{ValueOrExit(database::IntrinsicSelect(db.get(), intrinsic_id, camera_id), log)},
      extrinsic_{ValueOrExit(database::ExtrinsicSelect(db.get(), extrinsic_init_id, imu_id_, camera_id_), log)},
      gravity_{ValueOrExit(database::GravitySelect(db.get(), extrinsic_init_id), log)} {
    auto const time_handler{ValueOrExit(database::SplineInfoSelect(db.get(), spline_id, camera_id), log)};
    auto const control_points{database::ControlPointsSelect(db.get(), spline_id, camera_id)};

    spline_ = std::make_unique<spline::Se3Spline>(control_points, time_handler);
}

Hash VisualInertialOpt::CacheKey() const {
    return hashing::HashArguments(camera_info_, targets_, intrinsic_, imu_data_, spline_->ControlPoints(),
                                  spline_->GetTimeHandler().t0_ns_, spline_->GetTimeHandler().delta_t_ns_, extrinsic_,
                                  gravity_);
}

void VisualInertialOpt::Execute(StepId step_id, SqlitePtr const db) const {
    auto const [optimized_spline, optimized_extrinsic, optimized_gravity]{optimization::ExtrinsicOptimization(
        imu_data_, *spline_, extrinsic_, gravity_, camera_info_, targets_, intrinsic_, num_threads_)};

    // TODO(Jack): We also need a way to log the final and initial costs!
    log->info("{{{}, 'extrinsic': {}, 'gravity': [{:.3f}]}}", StepLogInfo{Type(), step_id}, optimized_extrinsic,
              fmt::join(optimized_gravity, ", "));

    database::SplineInfoInsert(db.get(), step_id, camera_id_, optimized_spline.GetTimeHandler());
    database::ControlPointsInsert(db.get(), step_id, camera_id_, optimized_spline.ControlPoints());
    database::GravityInsert(db.get(), step_id, optimized_gravity);

    // Diagnostic output - reprojection errors
    auto const ba_problem{
        optimization::SingleSplineCamProblem(camera_info_, intrinsic_, targets_, optimized_spline, camera_id_)};
    auto const residuals{optimization::EvaluateResiduals(ba_problem)};

    // TODO(Jack): One day if we adopt a spline optimization Result type we can add a transform function to RigState
    // here like we do for the regular bundle adjustment.
    transforms::RigState const rig_state{camera_id_, ba_problem.rig_poses,
                                         transforms::Extrinsics{{optimized_extrinsic}}};
    database::RigStateInsert(db.get(), step_id, targets_id_, rig_state);
    database::ReprojectionErrorsInsert(db.get(), step_id, targets_id_, residuals);

    // Diagnostic output - imu errors
    ImuErrors const imu_errors{
        optimization::EvaluateImuError(imu_data_, optimized_extrinsic, optimized_gravity, optimized_spline)};
    database::ImuErrorsInsert(db.get(), step_id, imu_data_id_, imu_id_, imu_errors);
}

}  // namespace reprojection::steps
