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
                                             StepId const spline_id, StepId const extrinsic_init_id, SqlitePtr const db)
    : camera_id_{camera_id},
      imu_id_{imu_id},
      targets_{database::ExtractedTargetsSelect(db.get(), targets_id, camera_id)},
      imu_data_{database::ImuDataSelect(db.get(), imu_data_id, imu_id)},
      num_threads_{num_threads} {
    if (auto const camera_info{database::CameraInfoSelect(db.get(), camera_info_id, camera_id)}) {
        camera_info_ = *camera_info;
    } else {
        log->error("{}", camera_info.error());
        std::exit(1);  // LCOV_EXCL_LINE
    }

    auto const intrinsics{database::IntrinsicSelect(db.get(), intrinsic_id, camera_id)};
    if (not intrinsics) {
        log->error("{{'intrinsic_id': '{}', 'asset_id': '{}', 'msg': 'Attempted to intrinsics but result was empty.'}}",
                   intrinsic_id.value, camera_id.value);
        std::exit(1);
    }
    intrinsics_ = *intrinsics;

    // TODO(Jack): Make these two functions private and provide one public helper that just loads the spline entirely.
    // This logic is repeated here and in the extrinsic init constructor.
    auto const time_handler{database::SplineInfoSelect(db.get(), spline_id, camera_id)};
    if (not time_handler) {
        log->error(
            "{{'spline_id': '{}', 'asset_id': '{}', 'msg': 'Attempted to spline time handler but result was "
            "empty.'}}",
            spline_id.value, camera_id.value);
        std::exit(1);
    }

    auto const control_points{database::ControlPointsSelect(db.get(), spline_id, camera_id)};
    spline_ = std::make_unique<spline::Se3Spline>(control_points, *time_handler);

    if (auto const extrinsic{database::ExtrinsicSelect(db.get(), extrinsic_init_id, imu_id_, camera_id_)}) {
        extrinsic_ = *extrinsic;
    } else {
        log->error("{}", extrinsic.error());
        std::exit(1);  // LCOV_EXCL_LINE
    }

    if (auto const gravity{database::GravitySelect(db.get(), extrinsic_init_id)}) {
        gravity_ = *gravity;
    } else {
        log->error("{}", gravity.error());
        std::exit(1);  // LCOV_EXCL_LINE
    }
}

Hash ExtrinsicOptimization::CacheKey() const {
    return hashing::HashArguments(camera_info_, targets_, intrinsics_, imu_data_, spline_->ControlPoints(),
                                  spline_->GetTimeHandler().t0_ns_, spline_->GetTimeHandler().delta_t_ns_, extrinsic_,
                                  gravity_);
}

void ExtrinsicOptimization::Execute(StepId step_id, SqlitePtr const db) const {
    auto const [optimized_spline, optimized_extrinsic, optimized_gravity]{optimization::ExtrinsicOptimization(
        imu_data_, *spline_, extrinsic_, gravity_, camera_info_, targets_, intrinsics_, num_threads_)};

    // TODO(Jack): We also need a way to log the final and initial costs!
    Array3d const optimized_gravity_fmt{optimized_gravity[0], optimized_gravity[1], optimized_gravity[2]};
    log->info("{{'step_id': {}, 'extrinsic': {{'asset_id_a': {}, 'asset_id_b' {}, 'se3_a_b' {}}}, 'gravity': {}}}",
              step_id.value, optimized_extrinsic.frame_a.value, optimized_extrinsic.frame_b.value,
              optimized_extrinsic.se3_a_b, optimized_gravity_fmt);

    database::SplineInfoInsert(db.get(), step_id, camera_id_, optimized_spline.GetTimeHandler());
    database::ControlPointsInsert(db.get(), step_id, camera_id_, optimized_spline.ControlPoints());
    database::ExtrinsicInsert(db.get(), step_id, optimized_extrinsic);
    database::GravityInsert(db.get(), step_id, optimized_gravity);
}

}  // namespace reprojection::steps
