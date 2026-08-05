#include "optimization/extrinsic_optimization.hpp"

#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "steps/extrinsic_optimization.hpp"

// TODO(Jack): unit test this step!

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

// TODO(Jack): This constructor, and all other step constructors like this are in dire need of a refactor!
ExtrinsicOptimization::ExtrinsicOptimization(AssetId const camera_id, AssetId const imu_id, StepId const targets_id,
                                             StepId const imu_data_id, int const num_threads,
                                             StepId const camera_info_id, StepId const intrinsic_id,
                                             StepId const spline_id, StepId const extrinsic_init_id,
                                             database::CalibrationDatabase const& db)
    : camera_id_{camera_id},
      imu_id_{imu_id},
      targets_{db.ExtractedTargetsSelect(targets_id, camera_id)},
      imu_data_{db.ImuDataSelect(imu_data_id, imu_id)},
      num_threads_{num_threads} {
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

    // TODO(Jack): Make these two functions private and provide one public helper that just loads the spline entirely.
    // This logic is repeated here and in the extrinsic init constructor.
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

    auto const extrinsic{db.ExtrinsicSelect(extrinsic_init_id, imu_id_, camera_id_)};
    if (not extrinsic) {
        log->error(
            "{{'extrinsic_init_id': '{}', 'imu_id': '{}', 'camera_id': '{}', 'msg': 'Attempted to load extrinsic but "
            "result "
            "was empty.'}}",
            extrinsic_init_id.value, imu_id.value, camera_id.value);
        std::exit(1);
    }
    extrinsic_ = *extrinsic;

    auto const gravity{db.GravitySelect(extrinsic_init_id)};
    if (not gravity) {
        log->error("{{'extrinsic_init_id': '{}', 'msg': 'Attempted to load gravity but result was empty.'}}",
                   extrinsic_init_id.value);
        std::exit(1);
    }
    gravity_ = *gravity;
}

Hash ExtrinsicOptimization::CacheKey() const {
    return hashing::HashArguments(camera_info_, targets_, intrinsics_, imu_data_, spline_->ControlPoints(),
                                  spline_->GetTimeHandler().t0_ns_, spline_->GetTimeHandler().delta_t_ns_, extrinsic_);
}

void ExtrinsicOptimization::Execute(StepId step_id, database::CalibrationDatabase const& db) const {
    auto const [optimized_spline, optimized_extrinsic, optimized_gravity]{optimization::ExtrinsicOptimization(
        imu_data_, *spline_, extrinsic_, gravity_, camera_info_, targets_, intrinsics_, num_threads_)};

    // TODO(Jack): We also need a way to log the final and initial costs!
    Array3d const optimized_gravity_fmt{optimized_gravity[0], optimized_gravity[1], optimized_gravity[2]};
    log->info("{{'step_id': {}, 'extrinsic': {{'asset_id_a': {}, 'asset_id_b' {}, 'se3_a_b' {}}}, 'gravity': {}}}",
              step_id.value, optimized_extrinsic.frame_a.value, optimized_extrinsic.frame_b.value,
              optimized_extrinsic.se3_a_b, optimized_gravity_fmt);

    db.SplineInfoInsert(step_id, camera_id_, optimized_spline.GetTimeHandler());
    db.ControlPointsInsert(step_id, camera_id_, optimized_spline.ControlPoints());
    db.ExtrinsicInsert(step_id, optimized_extrinsic);
    db.GravityInsert(step_id, optimized_gravity);
}

}  // namespace reprojection::steps
