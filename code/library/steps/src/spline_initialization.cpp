#include "spline/spline_initialization.hpp"

#include "calibration/calibration_utils.hpp"
#include "geometry/lie.hpp"
#include "hashing/hashing.hpp"
#include "logging/logging.hpp"
#include "optimization/extrinsic_optimization.hpp"
#include "spline/se3_spline.hpp"
#include "steps/spline_initialization.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

SplineInitialization::SplineInitialization(AssetId const camera_id, StepId const camera_poses_id,
                                           StepId const targets_id, StepId const camera_info_id,
                                           StepId const intrinsics_id, SqlitePtr const db)
    : camera_id_{camera_id},
      camera_poses_{database::CameraPosesSelect(db.get(), camera_poses_id, camera_id)},
      targets_id_{targets_id},
      targets_{database::ExtractedTargetsSelect(db.get(), targets_id, camera_id)} {
    if (auto const camera_info{database::CameraInfoSelect(db.get(), camera_info_id, camera_id)}) {
        camera_info_ = *camera_info;
    } else {
        log->error("{}", camera_info.error());
        std::exit(1);  // LCOV_EXCL_LINE
    }

    if (auto const intrinsics{database::IntrinsicSelect(db.get(), intrinsics_id, camera_id)}) {
        intrinsics_ = *intrinsics;
    } else {
        log->error("{}", intrinsics.error());
        std::exit(1);  // LCOV_EXCL_LINE
    }
}

Hash SplineInitialization::CacheKey() const {
    return hashing::HashArguments(camera_poses_, targets_, camera_info_, intrinsics_);
}

void SplineInitialization::Execute(StepId const step_id, SqlitePtr const db) const {
    auto const aligned_camera_poses{calibration::AlignRotations(camera_poses_)};

    // NOTE(Jack): We normally store our frames so that they transform a world point to the camera optical frame (ex.
    // bundle adjustment optimizes that directly). But the spline needs the inverse of that for its cumulative rotation
    // formulation to work and for the linear acceleration to be calculated in the desired frame by default.
    Frames invert_frames;
    for (auto const& [timestamp_ns, frame_i] : aligned_camera_poses) {
        invert_frames.insert({timestamp_ns, {geometry::Log(geometry::Exp(frame_i.pose).inverse())}});
    }

    // TODO(Jack): Parameterize frequency! Add to cache key probably?
    spline::Se3Spline const spline{spline::InitializeSe3SplineState(invert_frames, 100)};

    // TODO(Jack): Should we print out the time handler in more practical units than nanoseconds?
    log->info(
        "{{'step_id': {}, 'asset_id': {}, 'num_control_points': {}, 'time_handler': {{'t0_ns': {}, 'delta_t_ns': "
        "{}}}}}",
        step_id.value, camera_id_.value, spline.Size(), spline.GetTimeHandler().t0_ns_,
        spline.GetTimeHandler().delta_t_ns_);

    database::ControlPointsInsert(db.get(), step_id, camera_id_, spline.ControlPoints());
    database::SplineInfoInsert(db.get(), step_id, camera_id_, spline.GetTimeHandler());

    // Diagnostic output
    auto const [spline_poses,
                errors]{optimization::ReprojectionErrorSpline(camera_info_, targets_, intrinsics_, spline)};
    database::CameraPosesInsert(db.get(), step_id, targets_id_, camera_id_, spline_poses);
    database::ReprojectionErrorsInsert(db.get(), step_id, targets_id_, camera_id_, errors);
}

}  // namespace reprojection::steps
