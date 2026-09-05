#include "steps/extrinsic_cam_optimization.hpp"

#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "logging/logging.hpp"
#include "optimization/bundle_adjustment.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

using Ba = optimization::BundleAdjustment;

// TODO(Jack): Implicitly making the first values in the vectors the "reference" camera somehow leaving a lot up to
// fate. Is there some way we can formalize this role or is this already enough?
ExtrinsicCamOptimization::ExtrinsicCamOptimization(std::vector<CameraCalibration> const& camera_calibrations,
                                                   SqlitePtr db)
    : reference_camera_{camera_calibrations.front()} {
    // TODO(Jack): Out database was designed with the concept of "camera poses" and not "rig poses" so we need to do
    // some refactoring I think. Storing reference_camera_ as a class variable is a temporary solution for now.
    rig_poses_ =
        database::CameraPosesSelect(db.get(), reference_camera_.bundle_adjustment_id, reference_camera_.camera_id);

    cameras_.reserve(std::size(camera_calibrations));
    for (auto const& camera_calibration : camera_calibrations) {
        // TODO(Jack): Should we construct this directly in the vector and then refernece it instead of pushing it back
        // later?
        CameraProblemInput camera_input;
        camera_input.camera_id = camera_calibration.camera_id;

        if (auto const camera_info{
                database::CameraInfoSelect(db.get(), camera_calibration.camera_info_id, camera_input.camera_id)}) {
            camera_input.camera_info = *camera_info;
        } else {
            // TODO(Jack): This method of getting values back from the db is so ridiculously verbose it makes me want to
            // cry. Do something!
            log->error("{}", camera_info.error());
            std::exit(1);  // LCOV_EXCL_LINE
        }
        if (auto const intrinsics{
                database::IntrinsicSelect(db.get(), camera_calibration.bundle_adjustment_id, camera_input.camera_id)}) {
            camera_input.intrinsic = *intrinsics;
        } else {
            log->error("{}", intrinsics.error());
            std::exit(1);  // LCOV_EXCL_LINE
        }

        camera_input.targets = database::TargetsSelect(db.get(), camera_calibration.targets_id, camera_input.camera_id);

        // TODO(Jack): We also need to load the initialized extrinsic and the optimize intrinsic/extrinsic flags?

        cameras_.push_back(camera_input);
    }
}

Hash ExtrinsicCamOptimization::CacheKey() const {
    // TODO TODO TODO
    return hashing::HashArguments(1, 2, 3);
}

void ExtrinsicCamOptimization::Execute(StepId step_id, SqlitePtr const db) const {
    // TODO parameterize the sync delta!
    // TODO PASS NUM THREADS!
    Ba::Problem const problem{Ba::MultiCamProblem(cameras_, rig_poses_, 1000)};
    auto const [rig_poses, ceres_state, cameras]{Ba::Solve(problem, 1)};

    // TODO(Jack): We need to think about if we are really saving camera poses or rig poses!
    database::CameraPosesInsert(db.get(), step_id, reference_camera_.targets_id, reference_camera_.camera_id,
                                rig_poses);
}

}  // namespace reprojection::steps
