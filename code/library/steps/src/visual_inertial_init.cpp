#include "steps/visual_inertial_init.hpp"

#include "calibration/initialization_methods.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "optimization/extrinsic_optimization.hpp"

#include "utilities.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

VisualInertialInit::VisualInertialInit(AssetId const imu_id, StepId const imu_data_id, AssetId const cam_id,
                                       StepId const spline_id, int const num_threads, SqlitePtr const db)
    : imu_id_{imu_id},
      imu_data_id_{imu_data_id},
      imu_data_{database::ImuDataSelect(db.get(), imu_data_id, imu_id)},
      cam_id_{cam_id},
      spline_{std::make_unique<spline::Se3Spline>(
          database::ControlPointsSelect(db.get(), spline_id, cam_id),
          ValueOrExit(database::SplineInfoSelect(db.get(), spline_id, cam_id), log))},
      num_threads_{num_threads} {}

Hash VisualInertialInit::CacheKey() const {
    return hashing::HashArguments(imu_data_, spline_->ControlPoints(), spline_->GetTimeHandler().t0_ns_,
                                  spline_->GetTimeHandler().delta_t_ns_);
}

void VisualInertialInit::Execute(StepId const step_id, SqlitePtr const db) const {
    auto const [rotation_result, gravity_w]{calibration::EstimateCameraImuAlignment(*spline_, imu_data_, num_threads_)};

    // TODO(Jack): We should log these diagnostics like we did for the bundle adjustment!
    auto const [aa_imu_co, _]{rotation_result};
    // NOTE(Jack): In the cam-imu extrinsic initialization process we can only initialize the rotation so we just set
    // the translation to zero. If someone has an idea how to initialize the translation do tell!
    Array6d const tf_imu_co{aa_imu_co(0), aa_imu_co(1), aa_imu_co(2), 0, 0, 0};
    Extrinsic const extrinsic{imu_id_, cam_id_, tf_imu_co};

    log->info("{{{}, 'extrinsic': {}, 'gravity': [{:.3f}]}}", StepLogInfo{Type(), step_id}, extrinsic,
              fmt::join(gravity_w, ", "));

    database::ExtrinsicInsert(db.get(), step_id, extrinsic);
    database::GravityInsert(db.get(), step_id, gravity_w);

    // Diagnostic output.
    ImuErrors const errors{optimization::EvaluateImuError(imu_data_, extrinsic, gravity_w, *spline_)};
    database::ImuErrorsInsert(db.get(), step_id, imu_data_id_, imu_id_, errors);
}

}  // namespace reprojection::steps
