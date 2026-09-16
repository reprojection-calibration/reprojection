#include "optimization/bundle_adjustment.hpp"

#include <ceres/loss_function.h>

#include <ranges>

#include "cost_functions/reprojection_error.hpp"
#include "time_sync/time_sync.hpp"

namespace reprojection::optimization::bundle_adjustment {

// ERROR(Jack): What is a frame has too few valid pixels to actually constrain the pose? Should we entirely skip
// that frame? Or what if in general we have a minimum required of points per frame threshold?
std::pair<Discrete::Result, CeresState> Discrete::Solve(Problem const& problem, int const num_threads) {
    // TODO(Jack): It is a little messy how we construct the result from just part of the problem, and then iterate over
    // the problem below but ignore the part that we copied to the result and use the result instead. Really not the end
    // of the world but I feel like I am missing the plotline.
    Result result{problem};

    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR, num_threads};
    ceres::Problem ceres_problem{ceres_state.problem_options};

    for (auto const& [camera_id, _, frame_timestamp_ns, bundle] : problem.observations) {
        // cppcheck-suppress ignoredReturnValue
        auto const& [camera_info, _1, camera_options]{problem.cameras.at(camera_id)};
        auto& camera_state{result.camera_states.at(camera_id)};
        // Protect against the case of a missing rig pose - it can be that we have a observation for a frame where the
        // rig pose initialization was unsuccessful and we need to protect against that.
        if (not result.rig_poses.contains(frame_timestamp_ns)) {
            continue;  // LCOV_EXCL_LINE
        }
        auto& rig_pose{result.rig_poses.at(frame_timestamp_ns)};

        auto const& [pixels, points]{bundle};
        for (Eigen::Index j{0}; j < pixels.rows(); ++j) {
            ceres::CostFunction* const cost_function{
                cost_functions::Create(camera_info.camera_model, camera_info.bounds, pixels.row(j), points.row(j))};

            ceres_problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0),
                                           camera_state.intrinsic.value.data(), camera_state.extrinsic.data(),
                                           rig_pose.value.data());
        }

        if (not camera_options.optimize_intrinsic) {
            ceres_problem.SetParameterBlockConstant(camera_state.intrinsic.value.data());
        }
        if (not camera_options.optimize_extrinsic) {
            ceres_problem.SetParameterBlockConstant(camera_state.extrinsic.data());
        }
    }

    ceres::Solve(ceres_state.solver_options, &ceres_problem, &ceres_state.solver_summary);

    return {result, ceres_state};
}

Discrete::Problem Discrete::MultiCamProblem(AssetId const& cam0_id, Frames const& cam0_poses,
                                            std::vector<CameraProblemInput> const& cams,
                                            uint64_t const approx_sync_delta_ns) {
    Problem problem{cam0_id, cam0_poses};
    for (auto const& cam_i : cams) {
        AddCamera(cam_i, approx_sync_delta_ns, problem);
    }

    return problem;
}  // LCOV_EXCL_LINE

// TODO(Jack): Refactor this to use the AddCamera method! We repeate the observation iteration which is not so nice.
Discrete::Problem Discrete::SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                             TargetSamples const& targets, Frames const& frames,
                                             bool const optimize_intrinsic, AssetId const camera_id) {
    CameraProblemInput const cam0{
        camera_id, camera_info, intrinsic, targets, Array6d::Zero(), optimize_intrinsic, false,
    };

    // NOTE(Jack): Setting the sync tolerance to zero enforces exact matches only. Which considering that the frames
    // have to come from the camera's targets makes sense!
    return MultiCamProblem(cam0.camera_id, frames, {cam0}, 0);
}

Discrete::Problem Discrete::SingleFrameProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                               Bundle const& bundle, Pose const& pose, bool const optimize_intrinsic) {
    uint64_t constexpr timestamp_ns{0};
    ExtractedTarget const target{bundle, {}};
    AssetId const camera_id{0};

    return SingleCamProblem(camera_info, intrinsic, TargetSamples{{timestamp_ns, target}}, Frames{{timestamp_ns, pose}},
                            optimize_intrinsic, camera_id);
}

Continuous::Problem Continuous::MultiCamProblem(AssetId const& cam0_id, spline::Se3Spline const& rig_spline,
                                                Array6d const& se3_imu_rig, Vector3d const& gravity_w,
                                                std::vector<CameraProblemInput> const& cams) {
    Problem problem{cam0_id, rig_spline, se3_imu_rig, gravity_w};
    for (auto const& cam_i : cams) {
        AddCamera(cam_i, problem);
    }

    return problem;
}  // LCOV_EXCL_LINE

Continuous::Problem Continuous::SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                                 TargetSamples const& targets, spline::Se3Spline const& rig_spline,
                                                 Array6d const& se3_imu_rig, Vector3d const& gravity_w,
                                                 AssetId camera_id) {
    // NOTE(Jack): For the visual inertial extrinsic optimization we already have the intrinsic and cam extrinsic
    // results so we do not need to optimize these further.
    CameraProblemInput const cam0{
        camera_id, camera_info, intrinsic, targets, Array6d::Zero(), false, false,
    };

    return MultiCamProblem(cam0.camera_id, rig_spline, se3_imu_rig, gravity_w, {cam0});
}

// NOTE(Jack): All observations get synced to the rig_poses. This means that there might be poses that only have one
// target (i.e. the reference camera's target) and there might be non-reference camera observations that do not sync and
// are therefore never used. This is a simplifying assumption and does not cost us much but prevents us from have to
// implement a more intricate "changing reference camera" problem construction logic. Maybe we are just missing the
// abstraction to do that simply?
void Discrete::AddCamera(CameraProblemInput const& cam, uint64_t const approx_sync_delta_ns, Problem& problem) {
    problem.cameras.emplace(cam.camera_id, bundle_adjustment::Camera{cam.camera_info,  // LCOV_EXCL_LINE
                                                                     {cam.intrinsic, cam.extrinsic},
                                                                     {cam.optimize_intrinsic, cam.optimize_extrinsic}});

    auto const timestamps{cam.targets | std::views::keys};
    std::set<uint64_t> remaining_targets{std::cbegin(timestamps), std::cend(timestamps)};

    // TODO(Jack): We need to provide some information to the user regarding how the data was synced.
    for (auto const& [frame_timestamp_ns, _] : problem.rig_poses) {
        // TODO(Jack): Hand rolling the time synchronization logic here is not so nice, as we need it in multiple
        // places.
        auto const target_timestamps_it{time_sync::FindClosest(remaining_targets, frame_timestamp_ns)};
        if (target_timestamps_it == std::cend(remaining_targets)) {
            continue;  // LCOV_EXCL_LINE
        }

        auto const sample_timestamp_ns{*target_timestamps_it};
        if (not time_sync::IsWithinThreshold(sample_timestamp_ns, frame_timestamp_ns, approx_sync_delta_ns)) {
            continue;  // LCOV_EXCL_LINE
        }

        problem.observations.push_back(
            {cam.camera_id, sample_timestamp_ns, frame_timestamp_ns, cam.targets.at(sample_timestamp_ns).bundle});

        // Remove it so a double match cannot happen.
        remaining_targets.erase(target_timestamps_it);
    }
}

void Continuous::AddCamera(CameraProblemInput const& cam, Problem& problem) {
    problem.cameras.emplace(cam.camera_id, Camera{cam.camera_info,  // LCOV_EXCL_LINE
                                                  {cam.intrinsic, cam.extrinsic},
                                                  {cam.optimize_intrinsic, cam.optimize_extrinsic}});

    // NOTE(Jack): Here we do not need any time sync logic because we are using a spline! If the target it not found on
    // the spline then that will be handled during the actual problem construction.
    for (auto const& [timestamp_ns, target] : cam.targets) {
        problem.observations.push_back({cam.camera_id, timestamp_ns, timestamp_ns, target.bundle});
    }
}

transforms::RigState ToRigState(Discrete::Result const& result) {
    std::vector<Extrinsic> rig_cam_extrinsics;
    for (auto const& [camera_id, state_i] : result.camera_states) {
        // The Extrinsics() type does not allow self-connections/cycles!
        if (camera_id == result.rig_frame_asset_id) {
            continue;
        }

        // TODO(Jack): I do not know why this came as a little bit of a surprise to me that our extrinsic configuration
        // optimizes all the cameras with respect to the reference camera (see how rig_frame_asset_id is hardcoded
        // here). I think we need to think hard about this and make that is what we want! For some reason I thought it
        // would be that the extrinsic goes from neighbouring camera to neighbouring camera in a chain, but our current
        // setup is a tree. If we went from camera to camera I think this would make the optimization process ugly
        // because then we would need to chain them together and our cost function would have to accept a variable
        // number of extrinsics chained together! Does that make sense? We need to do some thinking here!!!
        Extrinsic const extrinsic_i{camera_id, result.rig_frame_asset_id, state_i.extrinsic};
        rig_cam_extrinsics.push_back(extrinsic_i);
    }

    return transforms::RigState{result.rig_frame_asset_id, result.rig_poses,
                                transforms::Extrinsics{{rig_cam_extrinsics}}};
}

std::vector<ReprojectionError> EvaluateResiduals(Discrete::Problem const& ba_problem) {
    std::vector<ReprojectionError> errors;
    errors.reserve(std::size(ba_problem.observations));

    for (auto const& [camera_id, sample_timestamp_ns, frame_timestamp_ns, bundle] : ba_problem.observations) {
        // cppcheck-suppress ignoredReturnValue
        auto const& [camera_info, camera_state, _]{ba_problem.cameras.at(camera_id)};
        if (not ba_problem.rig_poses.contains(frame_timestamp_ns)) {
            continue;
        }
        auto const& rig_pose{ba_problem.rig_poses.at(frame_timestamp_ns)};

        std::vector<double const*> parameter_blocks;
        parameter_blocks.push_back(camera_state.intrinsic.value.data());
        parameter_blocks.push_back(camera_state.extrinsic.data());
        parameter_blocks.push_back(rig_pose.value.data());

        auto const& [pixels, points]{bundle};

        // NOTE(Jack): Eigen is column major by default. Which means that if you just make a default array here and pass
        // the row pointer blindly into the EvaluateResidualBlock function it will not fill out the row but actually two
        // column elements! That is the reason why we have to specifically specify RowMajor here!
        Eigen::Array<double, Eigen::Dynamic, 2, Eigen::RowMajor> residuals_i{pixels.rows(), 2};
        for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
            ceres::CostFunction const* const cost_function{
                cost_functions::Create(camera_info.camera_model, camera_info.bounds, pixels.row(i), points.row(i))};

            cost_function->Evaluate(parameter_blocks.data(), residuals_i.row(i).data(), nullptr);

            // TODO(Jack): Should we use a smart pointer instead?
            delete cost_function;
        }

        errors.push_back({camera_id, sample_timestamp_ns, frame_timestamp_ns, residuals_i});
    }

    return errors;
}

}  // namespace reprojection::optimization::bundle_adjustment
