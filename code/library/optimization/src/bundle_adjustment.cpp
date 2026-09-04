#include "optimization/bundle_adjustment.hpp"

#include <ceres/loss_function.h>

#include "cost_functions/reprojection_error.hpp"

namespace reprojection::optimization {

// ERROR(Jack): What is a frame has too few valid pixels to actually constrain the pose? Should we entirely skip
// that frame? Or what if in general we have a minimum required of points per frame threshold?
BundleAdjustment::Result BundleAdjustment::Solve(Problem const& ba_problem, int const num_threads) {
    // TODO(Jack): It is a little messy how we construct the result from just part of the problem, and then iterate over
    // the problem below but ignore the part that we copied to the result and use the result instead. Really not the end
    // of the world but I feel like I am missing the plotline.
    Result result{ba_problem};
    result.ceres_state.solver_options.num_threads = num_threads;
    ceres::Problem ceres_problem{result.ceres_state.problem_options};

    for (auto const& [camera_id, timestamp_ns, bundle] : ba_problem.observations) {
        // cppcheck-suppress ignoredReturnValue
        auto const& [camera_info, _, camera_options]{ba_problem.cameras.at(camera_id)};
        auto& camera_state{result.camera_states.at(camera_id)};
        // Protect against the case of a missing rig pose - it can be that we have a observation for a frame where the
        // rig pose initialization was unsuccessful and we need to protect against that.
        if (not result.rig_poses.contains(timestamp_ns)) {
            continue;  // LCOV_EXCL_LINE
        }
        auto& rig_pose{result.rig_poses.at(timestamp_ns)};

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

    ceres::Solve(result.ceres_state.solver_options, &ceres_problem, &result.ceres_state.solver_summary);

    return result;
}

BundleAdjustment::Problem BundleAdjustment::SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                                             Frames const& frames, TargetSamples const& targets,
                                                             bool const optimize_intrinsic, AssetId const camera_id) {
    // For a single camera problem we do not consider the rig-camera extrinsic and set those to constant identity.
    Camera const camera{camera_info, CameraState{intrinsic, Array6d::Zero()}, CameraOptions{optimize_intrinsic, false}};

    // TODO(Jack): Should we do any check that the frame times match all the target times? Or is that something we need
    // to just check once when we actually construct the problem?
    std::vector<Observation> observations;
    for (auto const& [timestamp_ns, target] : targets) {
        observations.push_back({camera_id, timestamp_ns, target.bundle});
    }

    return {{{camera_id, camera}}, frames, observations};
}

BundleAdjustment::Problem BundleAdjustment::SingleCamProblem(CameraInfo const& camera_info, Intrinsic const& intrinsic,
                                                             Pose const& pose, Bundle const& bundle,
                                                             bool const optimize_intrinsic) {
    uint64_t constexpr timestamp_ns{0};
    ExtractedTarget const target{ExtractedTarget{bundle, {}}};
    AssetId const camera_id{0};

    return SingleCamProblem(camera_info, intrinsic, Frames{{timestamp_ns, pose}}, TargetSamples{{timestamp_ns, target}},
                            optimize_intrinsic, camera_id);
}

std::vector<ReprojectionError> EvaluateResiduals(BundleAdjustment::Problem const& ba_problem) {
    std::vector<ReprojectionError> errors;
    errors.reserve(std::size(ba_problem.observations));

    for (auto const& [camera_id, timestamp_ns, bundle] : ba_problem.observations) {
        auto const& [camera_info, camera_state, camera_options]{ba_problem.cameras.at(camera_id)};
        if (not ba_problem.rig_poses.contains(timestamp_ns)) {
            continue;
        }
        auto const& rig_pose{ba_problem.rig_poses.at(timestamp_ns)};

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

        errors.push_back({camera_id, timestamp_ns, residuals_i});
    }

    return errors;
}

}  // namespace  reprojection::optimization
