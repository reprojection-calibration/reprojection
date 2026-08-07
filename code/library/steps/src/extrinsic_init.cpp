#include "steps/extrinsic_init.hpp"

#include "calibration/initialization_methods.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "optimization/extrinsic_optimization.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

ExtrinsicInit::ExtrinsicInit(AssetId const camera_id, StepId const spline_id, AssetId const imu_id,
                             StepId const imu_data_id, int num_threads, SqlitePtr const db)
    : camera_id_{camera_id},
      imu_id_{imu_id},
      imu_data_id_{imu_data_id},
      imu_data_{database::ImuDataSelect(db.get(), imu_data_id, imu_id)},
      num_threads_{num_threads} {
    if (auto const time_handler{database::SplineInfoSelect(db.get(), spline_id, camera_id)}) {
        auto const control_points{database::ControlPointsSelect(db.get(), spline_id, camera_id)};

        spline_ = std::make_unique<spline::Se3Spline>(control_points, *time_handler);
    } else {
        log->error("{}", time_handler.error());
        std::exit(1);  // LCOV_EXCL_LINE
    }
}

Hash ExtrinsicInit::CacheKey() const {
    return hashing::HashArguments(imu_data_, spline_->ControlPoints(), spline_->GetTimeHandler().t0_ns_,
                                  spline_->GetTimeHandler().delta_t_ns_);
}

void ExtrinsicInit::Execute(StepId const step_id, SqlitePtr const db) const {
    auto const [rotation_result, gravity_w]{calibration::EstimateCameraImuAlignment(*spline_, imu_data_, num_threads_)};

    // TODO(Jack): We should log these diagnostics like we did for the bundle adjustment!
    auto const [aa_imu_co, _]{rotation_result};
    // NOTE(Jack): In the cam-imu extrinsic initialization process we can only initialize the rotation so we just set
    // the translation to zero. If someone has an idea how to initialize the translation do tell!
    Array6d const tf_imu_co{aa_imu_co(0), aa_imu_co(1), aa_imu_co(2), 0, 0, 0};
    Extrinsic const extrinsic{imu_id_, camera_id_, tf_imu_co};

    // TODO(Jack): For some reason our format function does not work with the original matrix/vector type so we manually
    // convert it to an array here.
    Array3d const gravity_w_fmt{gravity_w[0], gravity_w[1], gravity_w[2]};
    log->info("{{'step_id': {}, 'extrinsic': {{'asset_id_a': {}, 'asset_id_b' {}, 'se3_a_b' {}}}, 'gravity': {}}}",
              step_id.value, extrinsic.frame_a.value, extrinsic.frame_b.value, extrinsic.se3_a_b, gravity_w_fmt);

    database::ExtrinsicInsert(db.get(), step_id, extrinsic);
    database::GravityInsert(db.get(), step_id, gravity_w);

    // Diagnostic output.
    ImuErrors const errors{optimization::EvaluateImuError(imu_data_, extrinsic, gravity_w, *spline_)};
    database::ImuErrorsInsert(db.get(), step_id, imu_data_id_, imu_id_, errors);
}

}  // namespace reprojection::steps
