#include "steps/extrinsic_init.hpp"

#include "calibration/initialization_methods.hpp"
#include "hashing/hashing.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

ExtrinsicInit::ExtrinsicInit(AssetId camera_id, StepId spline_id, AssetId imu_id, StepId imu_data_id, int num_threads,
                             database::CalibrationDatabase const& db)
    : camera_id_{camera_id},
      imu_id_{imu_id},
      imu_data_{db.ImuDataSelect(imu_data_id, imu_id)},
      num_threads_{num_threads} {
    // TODO(Jack): Copy and pasted practically verbatim from the intrinsic init. Also copy and pasted almost identically
    // below. Seems like this is a good place for a templated helper function.
    auto const time_handler{db.SplineInfoSelect(spline_id, camera_id)};
    if (not time_handler) {
        log->error(
            "{{'spline_id': '{}', 'asset_id': '{}', 'msg': 'Attempted to spline time handler but result was "
            "empty.'}}",
            spline_id.value, camera_id.value);
        std::exit(1);
    }

    auto const control_points{db.ControlPointsSelect(spline_id, camera_id)};
    spline_ = std::make_unique<spline::Se3Spline>(control_points, *time_handler);
}

Hash ExtrinsicInit::CacheKey() const {
    return hashing::HashArguments(imu_data_, spline_->ControlPoints(), spline_->GetTimeHandler().t0_ns_,
                                  spline_->GetTimeHandler().delta_t_ns_);
}

void ExtrinsicInit::Execute(StepId step_id, database::CalibrationDatabase& db) const {
    auto const [rotation_result, gravity_w]{calibration::EstimateCameraImuAlignment(*spline_, imu_data_, num_threads_)};

    // TODO(Jack): Should we do something with the diagnostics? There are several places now where we ignore the
    // returned optimization diagnostics but I am sure that a user would appreciate these in the database.
    auto const [aa_imu_co, _]{rotation_result};

    // NOTE(Jack): In the cam-imu extrinsic initialization process we can only initialize the rotation so we just set
    // the translation to zero. If someone has an idea how to initialize the translation do tell!
    Array6d const tf_imu_co{aa_imu_co(0), aa_imu_co(1), aa_imu_co(2), 0, 0, 0};

    database::Extrinsic2 const extrinsic{imu_id_, camera_id_, tf_imu_co};
    db.ExtrinsicInsert(step_id, extrinsic);
    db.GravityInsert(step_id, gravity_w);
}

}  // namespace reprojection::steps
