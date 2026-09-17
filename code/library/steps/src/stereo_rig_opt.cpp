#include "steps/stereo_rig_opt.hpp"

#include "database/calib_db.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "optimization/bundle_adjustment.hpp"

#include "utilities.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

StereoRigOpt::StereoRigOpt(AssetId const cam0_id, std::vector<CamStageIds> const& cams, StepId const extrinsic_init_id,
                           int const num_threads, uint64_t const approx_sync_delta_ns, SqlitePtr db)
    : cam0_id_{cam0_id}, num_threads_{num_threads}, approx_sync_delta_ns_{approx_sync_delta_ns} {
    ba_input_.reserve(std::size(cams));
    for (auto const& cam_i : cams) {
        cam_target_ids_.emplace(cam_i.asset_id, cam_i.targets_id);

        bool const is_reference_cam{cam_i.asset_id == cam0_id_};
        if (is_reference_cam) {
            rig_poses_ = database::CameraPosesSelect(db.get(), cam_i.bundle_adjustment_id, cam_i.asset_id);
        }

        auto const camera_info{
            ValueOrExit(database::CameraInfoSelect(db.get(), cam_i.camera_info_id, cam_i.asset_id), log)};
        auto const intrinsic{
            ValueOrExit(database::IntrinsicSelect(db.get(), cam_i.bundle_adjustment_id, cam_i.asset_id), log)};
        auto const targets{database::TargetsSelect(db.get(), cam_i.targets_id, cam_i.asset_id)};
        auto const extrinsic{
            ValueOrExit(database::ExtrinsicSelect(db.get(), extrinsic_init_id, cam_i.asset_id, cam0_id_), log)};

        ba_input_.emplace_back(CameraProblemInput{cam_i.asset_id, camera_info, intrinsic, targets, extrinsic.se3_a_b,
                                                  false, not is_reference_cam});
    }
}

Hash StereoRigOpt::CacheKey() const {
    Hash const initial_hash{hashing::HashArgs(rig_poses_, approx_sync_delta_ns_)};

    // TODO(Jack): If we end up keeping the CameraProblemInput type we should add a serialize function for it directly!
    return std::ranges::fold_left(ba_input_, initial_hash, [](Hash const& hash, auto const& camera) {
        return hashing::HashArgs(hash, camera.camera_info, camera.intrinsic, camera.targets, camera.extrinsic,
                                 camera.optimize_intrinsic, camera.optimize_extrinsic);
    });
}

void StereoRigOpt::Execute(StepId step_id, SqlitePtr const db) const {
    using Discrete = optimization::Discrete;

    Discrete::Problem const problem{Discrete::MultiCamProblem(cam0_id_, rig_poses_, ba_input_, approx_sync_delta_ns_)};
    auto const [result, ceres_state]{Discrete::Solve(problem, num_threads_)};

    database::RigStateInsert(db.get(), step_id, cam_target_ids_.at(cam0_id_), ToRigState(result));

    Discrete::Problem const optimized_problem{problem, result.rig.frames, result.camera_states};
    auto const errors{EvaluateResiduals(optimized_problem)};
    database::ReprojectionErrorsInsert(db.get(), step_id, cam_target_ids_, errors);

    log->info("{{{}, 'problem': {}}}}}", StepLogInfo{Type(), step_id}, problem);
    log->info("{{{}, 'result': {}, 'solver_summary': {}}}}}", StepLogInfo{Type(), step_id}, result,
              ceres_state.solver_summary);
}

}  // namespace reprojection::steps
