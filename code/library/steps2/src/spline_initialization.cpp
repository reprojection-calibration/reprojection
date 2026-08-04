#include "spline/spline_initialization.hpp"

#include "geometry/lie.hpp"
#include "hashing/hashing.hpp"
#include "spline/se3_spline.hpp"
#include "steps/spline_initialization.hpp"

namespace reprojection::steps {

SplineInitialization::SplineInitialization(AssetId const camera_id, StepId const camera_poses_id,
                                           database::CalibrationDatabase const& db)
    : camera_id_{camera_id}, camera_poses_{db.CameraPosesSelect(camera_poses_id, camera_id)} {}

Hash SplineInitialization::CacheKey() const { return hashing::HashArguments(camera_poses_); }

void SplineInitialization::Execute(StepId const step_id, database::CalibrationDatabase& db) const {
    // NOTE(Jack): We normally store our frames so that they transform a world point to the camera optical frame (ex.
    // bundle adjustment optimizes that directly). But the spline needs the inverse of that for its cumulative rotation
    // formulation to work and for the linear acceleration to be calculated in the desired frame by default.
    Frames invert_frames;
    for (auto const& [timestamp_ns, frame_i] : camera_poses_) {
        invert_frames.insert({timestamp_ns, {geometry::Log(geometry::Exp(frame_i.pose).inverse())}});
    }

    // TODO(Jack): Parameterize frequency! Add to cache key probably?
    spline::Se3Spline const spline{spline::InitializeSe3SplineState(invert_frames, 100)};

    db.ControlPointsInsert(step_id, camera_id_, spline.ControlPoints());
    db.SplineInfoInsert(step_id, camera_id_, spline.GetTimeHandler());
}

}  // namespace reprojection::steps
