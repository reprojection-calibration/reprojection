#include "steps/stereo_rig_opt.hpp"

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
StereoRigOpt::StereoRigOpt(std::vector<CamStageIds> const& cams, StepId const extrinsic_init_id, int const num_threads,
                           uint64_t const approx_sync_delta_ns, SqlitePtr db)
    : cam0_{cams.front()}, num_threads_{num_threads}, approx_sync_delta_ns_{approx_sync_delta_ns} {
    rig_poses_ = database::CameraPosesSelect(db.get(), cam0_.bundle_adjustment_id, cam0_.asset_id);

    ba_input_.reserve(std::size(cams));
    for (auto const& cam_i : cams) {
        bool const is_reference_cam{cam_i.asset_id == cam0_.asset_id};

        // TODO(Jack): This is really hard to read, but the basic idea is if the camera is the reference camera then
        // hardcode the extrinsic identity and do not optimize it. Otherwise load and optimize the extrinsic.
        optimization::CameraProblemInput const ba_input_i{
            cam_i.asset_id,
            ValueOrExit(database::CameraInfoSelect(db.get(), cam_i.camera_info_id, ba_input_i.camera_id), log),
            ValueOrExit(database::IntrinsicSelect(db.get(), cam_i.bundle_adjustment_id, ba_input_i.camera_id), log),
            database::TargetsSelect(db.get(), cam_i.targets_id, ba_input_i.camera_id),
            is_reference_cam
                ? Array6d::Zero()
                : ValueOrExit(
                      database::ExtrinsicSelect(db.get(), extrinsic_init_id, ba_input_i.camera_id, cam0_.asset_id), log)
                      .se3_a_b,
            false,
            not is_reference_cam};

        ba_input_.push_back(ba_input_i);
    }
}

Hash StereoRigOpt::CacheKey() const {
    Hash const initial_hash{hashing::HashArguments(rig_poses_, approx_sync_delta_ns_)};

    // TODO(Jack): If we end up keeping the CameraProblemInput type we should add a serialize function for it directly!
    return std::ranges::fold_left(ba_input_, initial_hash, [](Hash const& hash, auto const& camera) {
        return hashing::HashArguments(hash, camera.camera_info, camera.intrinsic, camera.targets, camera.extrinsic,
                                      camera.optimize_intrinsic, camera.optimize_extrinsic);
    });
}

void StereoRigOpt::Execute(StepId step_id, SqlitePtr const db) const {
    // ERROR(Jack): Parameterize the sync delta!
    // TODO(Jack): Pass number of threads!
    // NOTE(Jack): Here an important things happens and that is that we pick the first camera in cameras_ as the
    // reference camera. For now we can implicitly do this but it might turn out one day that we need to be explicit
    // here to make sure we are consistent with the cam-imu extrinsic calibration too. I am not really a 'span' api
    // user, but we use it here to pass all the non-reference cameras.
    Ba::Problem const problem{
        Ba::MultiCamProblem(ba_input_.front(), rig_poses_, std::span{ba_input_}.subspan(1), approx_sync_delta_ns_)};
    auto const [result, ceres_state]{Ba::Solve(problem, num_threads_)};

    database::RigStateInsert(db.get(), step_id, cam0_.targets_id, optimization::ToRigState(result));

    // TODO(Jack): As we are basically just doing another bundle adjustment here we should also write the reprojection
    // errors! The only problem is that the current reprojection error database interface does not handle the new rig
    // idea.

    log->info("{{{}, 'problem': {}}}}}", StepLogInfo{Type(), step_id}, problem);
    log->info("{{{}, 'result': {}, 'solver_summary': {}}}}}", StepLogInfo{Type(), step_id}, result,
              ceres_state.solver_summary);
}

}  // namespace reprojection::steps
