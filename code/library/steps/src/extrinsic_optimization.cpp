#include "optimization/extrinsic_optimization.hpp"

#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "steps/extrinsic_optimization.hpp"

// ERROR(Jack): We really really need to test this step! It is just so complicated and I got lazy during a the huge
// workflow refactor.

namespace reprojection::steps {

// LCOV_EXCL_START

namespace {

auto const log{logging::Get("steps")};

}

ExtrinsicOptimization::ExtrinsicOptimization(AssetId const camera_id, AssetId const imu_id, StepId const targets_id,
                                             StepId const imu_data_id, int const num_threads,
                                             StepId const camera_info_id, StepId const intrinsic_id,
                                             StepId const spline_id, StepId const extrinsic_init_id, SqlitePtr const db)
    : camera_id_{camera_id},
      imu_id_{imu_id},
      targets_id_{targets_id},
      targets_{database::ExtractedTargetsSelect(db.get(), targets_id, camera_id)},
      imu_data_id_{imu_data_id},
      imu_data_{database::ImuDataSelect(db.get(), imu_data_id, imu_id)},
      num_threads_{num_threads} {
    // TODO(Jack): Is there not a better "looking" way to load values from the databases? Nothing technically wrong
    // here, I think the higher level problem is that the extrinsic optimization depends on so much information that we
    // need load so many things regardless of how it looks/works.
    if (auto const camera_info{database::CameraInfoSelect(db.get(), camera_info_id, camera_id)}) {
        camera_info_ = *camera_info;
    } else {
        log->error("{}", camera_info.error());
        std::exit(1);  // LCOV_EXCL_LINE
    }

    if (auto const intrinsics{database::IntrinsicSelect(db.get(), intrinsic_id, camera_id)}) {
        intrinsics_ = *intrinsics;
    } else {
        log->error("{}", intrinsics.error());
        std::exit(1);  // LCOV_EXCL_LINE
    }

    if (auto const time_handler{database::SplineInfoSelect(db.get(), spline_id, camera_id)}) {
        auto const control_points{database::ControlPointsSelect(db.get(), spline_id, camera_id)};

        spline_ = std::make_unique<spline::Se3Spline>(control_points, *time_handler);
    } else {
        log->error("{}", time_handler.error());
        std::exit(1);  // LCOV_EXCL_LINE
    }

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

    // Diagnostic output - reprojection errors
    auto const [spline_poses, reprojection_errors]{
        optimization::ReprojectionErrorSpline(camera_info_, targets_, intrinsics_, optimized_spline)};
    database::CameraPosesInsert(db.get(), step_id, targets_id_, camera_id_, spline_poses);
    database::ReprojectionErrorsInsert(db.get(), step_id, targets_id_, camera_id_, reprojection_errors);

    // Diagnostic output - imu errors
    ImuErrors const imu_errors{
        optimization::EvaluateImuError(imu_data_, optimized_extrinsic, optimized_gravity, optimized_spline)};
    database::ImuErrorsInsert(db.get(), step_id, imu_data_id_, imu_id_, imu_errors);
}

// LCOV_EXCL_STOP

}  // namespace reprojection::steps
