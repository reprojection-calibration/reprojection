#include "steps/cam_cam_extrinsic_optimization.hpp"

#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "optimization/bundle_adjustment.hpp"

#include "utilities.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

using Ba = optimization::BundleAdjustment;

// TODO(Jack): Implicitly making the first values in the vectors the "reference" camera somehow leaving a lot up to
// fate. Is there some way we can formalize this role?
CamCamExtrinsicOptimization::CamCamExtrinsicOptimization(std::vector<CamStageIds> const& camera_calibrations,
                                                         StepId const extrinsic_init_id, int const num_threads,
                                                         uint64_t const approx_sync_delta_ns, SqlitePtr db)
    : reference_camera_{camera_calibrations.front()},
      num_threads_{num_threads},
      approx_sync_delta_ns_{approx_sync_delta_ns} {
    // TODO(Jack): Out database was designed with the concept of "camera poses" and not "rig poses" so we need to do
    // some refactoring I think. Storing reference_camera_ as a class variable is a temporary solution for now.
    rig_poses_ =
        database::CameraPosesSelect(db.get(), reference_camera_.bundle_adjustment_id, reference_camera_.camera_id);

    cameras_.reserve(std::size(camera_calibrations));
    for (auto const& calib_i : camera_calibrations) {
        // TODO(Jack): Should we construct this directly in the vector and then reference it instead of pushing it back
        // later?
        optimization::CameraProblemInput cam_i;
        cam_i.camera_id = calib_i.camera_id;
        cam_i.optimize_intrinsic = false;

        cam_i.camera_info =
            ValueOrExit(database::CameraInfoSelect(db.get(), calib_i.camera_info_id, cam_i.camera_id), log);
        cam_i.intrinsic =
            ValueOrExit(database::IntrinsicSelect(db.get(), calib_i.bundle_adjustment_id, cam_i.camera_id), log);

        // ERROR(Jack): This is a mega hack coming from out current inconsistency around how to handle the identity
        // extrinsic. We need to uniformly solve this as already noted elsewhere.
        if (cam_i.camera_id == reference_camera_.camera_id) {
            cam_i.extrinsic = Array6d::Zero();
            cam_i.optimize_extrinsic = false;
        } else {
            auto const extrinsic_i{ValueOrExit(
                database::ExtrinsicSelect(db.get(), extrinsic_init_id, cam_i.camera_id, reference_camera_.camera_id),
                log)};

            cam_i.extrinsic = extrinsic_i.se3_a_b;
            cam_i.optimize_extrinsic = true;
        }

        cam_i.targets = database::TargetsSelect(db.get(), calib_i.targets_id, cam_i.camera_id);

        cameras_.push_back(cam_i);
    }

    // TODO(Jack): Should we throw and error here if cameras_ is empty?
}

Hash CamCamExtrinsicOptimization::CacheKey() const {
    Hash const initial_hash{hashing::HashArguments(rig_poses_, approx_sync_delta_ns_)};

    // TODO(Jack): If we end up keeping the CameraProblemInput type we should add a serialize function for it directly!
    return std::ranges::fold_left(cameras_, initial_hash, [](Hash const& hash, auto const& camera) {
        return hashing::HashArguments(hash, camera.camera_info, camera.intrinsic, camera.targets, camera.extrinsic,
                                      camera.optimize_intrinsic, camera.optimize_extrinsic);
    });
}

void CamCamExtrinsicOptimization::Execute(StepId step_id, SqlitePtr const db) const {
    // ERROR(Jack): Parameterize the sync delta!
    // TODO(Jack): Pass number of threads!
    // NOTE(Jack): Here an important things happens and that is that we pick the first camera in cameras_ as the
    // reference camera. For now we can implicitly do this but it might turn out one day that we need to be explicit
    // here to make sure we are consistent with the cam-imu extrinsic calibration too. I am not really a 'span' api
    // user, but we use it here to pass all the non-reference cameras.
    Ba::Problem const problem{
        Ba::MultiCamProblem(cameras_.front(), rig_poses_, std::span{cameras_}.subspan(1), approx_sync_delta_ns_)};
    auto const [result, ceres_state]{Ba::Solve(problem, num_threads_)};

    database::RigStateInsert(db.get(), step_id, reference_camera_.targets_id, optimization::ToRigState(result));

    // TODO(Jack): As we are basically just doing another bundle adjustment here we should also write the reprojection
    // errors! The only problem is that the current reprojection error database interface does not handle the new rig
    // idea.

    log->info("{{{}, 'problem': {}}}}}", StepLogInfo{Type(), step_id}, problem);
    log->info("{{{}, 'result': {}, 'solver_summary': {}}}}}", StepLogInfo{Type(), step_id}, result,
              ceres_state.solver_summary);
}

}  // namespace reprojection::steps
