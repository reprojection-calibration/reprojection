#include "steps/visual_inertial_opt.hpp"

#include "database/calib_db.hpp"
#include "hashing/hashing.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "optimization/visual_inertial.hpp"

#include "utilities.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

using VisualInertial = optimization::VisualInertial;

VisualInertialOpt::VisualInertialOpt(AssetId const imu_id, StepId const imu_data_id, AssetId const cam0_id,
                                     std::vector<CamStageIds> const& cams, StepId const spline_id,
                                     StepId const vi_extrinsic_init_id, StepId const stereo_rig_opt_id,
                                     int const num_threads, SqlitePtr const db)
    : imu_id_{imu_id},
      imu_data_id_{imu_data_id},
      imu_data_{database::ImuDataSelect(db.get(), imu_data_id, imu_id)},
      cam0_id_{cam0_id},
      spline_{std::make_unique<spline::Se3Spline>(
          database::ControlPointsSelect(db.get(), spline_id, cam0_id),
          ValueOrExit(database::SplineInfoSelect(db.get(), spline_id, cam0_id), log))},
      extrinsic_imu_rig_{ValueOrExit(database::ExtrinsicSelect(db.get(), vi_extrinsic_init_id, imu_id, cam0_id), log)},
      gravity_{ValueOrExit(database::GravitySelect(db.get(), vi_extrinsic_init_id), log)},
      num_threads_{num_threads} {
    // TODO(Jack): This loading loop is copied almost verbatim (except the optimization boolean flags and frame
    // loading). Can we avoid the copy and paste?
    ba_input_.reserve(std::size(cams));
    for (auto const& cam_i : cams) {
        cam_target_ids_.emplace(cam_i.asset_id, cam_i.targets_id);

        auto const camera_info{
            ValueOrExit(database::CameraInfoSelect(db.get(), cam_i.camera_info_id, cam_i.asset_id), log)};
        auto const intrinsic{
            ValueOrExit(database::IntrinsicSelect(db.get(), cam_i.bundle_adjustment_id, cam_i.asset_id), log)};
        auto const targets{database::TargetsSelect(db.get(), cam_i.targets_id, cam_i.asset_id)};

        // WARN(Jack): This is a super hack! What this does is protect us against the monocular camera case where the
        // 'stereo_rig_opt_id' is not actually initialized. We need a better programmatic way to deal with this. This
        // hack here does not scale or cover the case one day when we actually support rig/cam0 non-identity extrinsics.
        Extrinsic extrinsic;
        if (cam_i.asset_id == cam0_id_) {
            extrinsic = Extrinsic(cam_i.asset_id, cam0_id_, Array6d::Zero());
        } else {
            extrinsic =
                ValueOrExit(database::ExtrinsicSelect(db.get(), stereo_rig_opt_id, cam_i.asset_id, cam0_id_), log);
        }

        ba_input_.emplace_back(
            CameraProblemInput{cam_i.asset_id, camera_info, intrinsic, targets, extrinsic.se3_a_b, false, false});
    }
}

Hash VisualInertialOpt::CacheKey() const {
    Hash const initial_hash{hashing::HashArgs(imu_data_, spline_->ControlPoints(), spline_->GetTimeHandler().t0_ns_,
                                              spline_->GetTimeHandler().delta_t_ns_, extrinsic_imu_rig_, gravity_)};

    // TODO(Jack): Logic copy and pasted from stereo opt step.
    // TODO(Jack): If we end up keeping the CameraProblemInput type we should add a serialize function for it directly!
    return std::ranges::fold_left(ba_input_, initial_hash, [](Hash const& hash, auto const& camera) {
        return hashing::HashArgs(hash, camera.camera_info, camera.intrinsic, camera.targets, camera.extrinsic,
                                 camera.optimize_intrinsic, camera.optimize_extrinsic);
    });
}

void VisualInertialOpt::Execute(StepId step_id, SqlitePtr const db) const {
    auto const problem{VisualInertial::MultiCamProblem(cam0_id_, *spline_, extrinsic_imu_rig_.se3_a_b, gravity_,
                                                       ba_input_, imu_data_)};
    auto const [result, ceres_state]{optimization::VisualInertial::Solve(problem, num_threads_)};

    // TOOD(Jack): Should we add a constructor/factory that just lets us update the se3 part?
    Extrinsic const extrinsic_imu_rig{imu_id_, cam0_id_, result.inertial_state.se3_imu_rig};
    log->info("{{{}, 'extrinsic': {}, 'gravity': [{:.3f}], 'solver_summary': {}}}", StepLogInfo{Type(), step_id},
              extrinsic_imu_rig, fmt::join(result.inertial_state.gravity_w, ", "), ceres_state.solver_summary);

    database::SplineInfoInsert(db.get(), step_id, cam0_id_, result.rig.spline.GetTimeHandler());
    database::ControlPointsInsert(db.get(), step_id, cam0_id_, result.rig.spline.ControlPoints());
    database::GravityInsert(db.get(), step_id, result.inertial_state.gravity_w);

    // Diagnostic output - reprojection errors
    VisualInertial::Problem const optimized_problem{problem, result.rig, result.inertial_state, result.camera_states};
    auto const ba_problem{optimization::ToBaProblem(optimized_problem)};
    auto const residuals{optimization::EvaluateResiduals(ba_problem)};

    // TODO(Jack): One day if we adopt a spline optimization Result type we can add a transform function to RigState
    // here like we do for the regular bundle adjustment.
    transforms::RigState const rig_state{cam0_id_, ba_problem.rig.frames, transforms::Extrinsics{{extrinsic_imu_rig}}};
    database::RigStateInsert(db.get(), step_id, cam_target_ids_.at(cam0_id_), rig_state);
    database::ReprojectionErrorsInsert(db.get(), step_id, cam_target_ids_, residuals);

    // Diagnostic output - imu errors
    ImuErrors const imu_errors{optimization::EvaluateImuError(imu_data_, extrinsic_imu_rig,
                                                              result.inertial_state.gravity_w, result.rig.spline)};
    database::ImuErrorsInsert(db.get(), step_id, imu_data_id_, imu_id_, imu_errors);
}

}  // namespace reprojection::steps
