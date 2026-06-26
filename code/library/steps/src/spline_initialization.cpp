#include "spline/spline_initialization.hpp"

#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"
#include "optimization/extrinsic_optimization.hpp"
#include "steps/spline_initialization.hpp"  // This should be the first header!

namespace reprojection::steps {

std::string SplineInitialization::HashInputs() const { return hashing::HashArguments(camera_info_, targets_, bundle_); }

spline::Se3Spline SplineInitialization::Compute() const {
    Frames invert_frames;
    for (auto const& [timestamp_ns, frame_i] : bundle_.frames) {
        invert_frames.insert({timestamp_ns, {geometry::Log(geometry::Exp(frame_i.pose).inverse())}});
    }

    // TODO(Jack): Parameterize frequency! Add to cache key probably?
    spline::Se3Spline const spline{spline::InitializeSe3SplineState(invert_frames, 100)};

    return spline;
}

spline::Se3Spline SplineInitialization::Load(SqlitePtr const db) const {
    auto const control_points{database::ReadControlPoints(db, EntityId(), StepType())};
    auto const time_handler{database::ReadTimeHandler(db, EntityId(), StepType())};

    if (not time_handler) {
        std::cout << "WE NEED AN ERROR STRATEGY! SplineInitialization::Load()" << std::endl;  // LCOV_EXCL_LINE
    }

    return spline::Se3Spline{control_points, *time_handler};
}

void SplineInitialization::Save(spline::Se3Spline const& spline, SqlitePtr const db) const {
    database::InsertControlPoints(db, EntityId(), StepType(), spline.ControlPoints());
    database::InsertTimeHandler(db, EntityId(), StepType(), spline.GetTimeHandler());

    auto const [spline_poses,
                errors]{optimization::ReprojectionErrorSpline(camera_info_, targets_, bundle_.camera_state, spline)};
    database::InsertPoses(db, EntityId(), StepType(), spline_poses);
    database::InsertReprojectionErrors(db, EntityId(), StepType(), errors);
}

}  // namespace reprojection::steps
