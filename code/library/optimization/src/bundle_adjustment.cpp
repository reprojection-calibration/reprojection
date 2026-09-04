#include "optimization/bundle_adjustment.hpp"

#include <ceres/loss_function.h>

#include "cost_functions/reprojection_error.hpp"

namespace reprojection::optimization {

// ERROR(Jack): What is a frame has too few valid pixels to actually constrain the pose? Should we entirely skip
// that frame? Or what if in general we have a minimum required of points per frame threshold?
BaResult BundleAdjustment(BaProblem const& ba_problem, int const num_threads) {
    // TODO(Jack): It is a little messy how we construct the result from just part of the problem, and then iterate over
    // the problem below but ignore the part that we copied to the result and use the result instead. Really not the end
    // of the world but I feel like I am missing the plotline.
    BaResult result{ba_problem};
    result.ceres_state.solver_options.num_threads = num_threads;
    ceres::Problem ceres_problem{result.ceres_state.problem_options};

    for (auto const& [camera_id, timestamp_ns, bundle] : ba_problem.observations) {
        // TODO(Jack): We need to protect against bad .at() access here!
        // cppcheck-suppress ignoredReturnValue
        auto const& [camera_info, _, camera_options]{ba_problem.cameras.at(camera_id)};
        auto& camera_state{result.camera_states.at(camera_id)};
        auto& frame{result.frames.at(timestamp_ns)};

        auto const& [pixels, points]{bundle};
        for (Eigen::Index j{0}; j < pixels.rows(); ++j) {
            ceres::CostFunction* const cost_function{
                cost_functions::Create(camera_info.camera_model, camera_info.bounds, pixels.row(j), points.row(j))};

            ceres_problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0),
                                           camera_state.intrinsic.intrinsics.data(), frame.pose.data());
        }

        if (not camera_options.optimize_intrinsic) {
            ceres_problem.SetParameterBlockConstant(camera_state.intrinsic.intrinsics.data());
        }
    }

    ceres::Solve(result.ceres_state.solver_options, &ceres_problem, &result.ceres_state.solver_summary);

    return result;
}

ReprojectionErrors ReprojectionError(CameraInfo const& sensor, CameraMeasurements const& targets,
                                     OptimizationState const& state) {
    ReprojectionErrors residuals;
    for (auto const& [timestamp_ns, frame_i] : state.frames) {
        auto const& [pixels, points]{targets.at(timestamp_ns).bundle};

        std::vector<double const*> parameter_blocks;
        parameter_blocks.push_back(state.camera_state.intrinsics.data());
        parameter_blocks.push_back(frame_i.pose.data());

        // NOTE(Jack): Eigen is column major by default. Which means that if you just make a default array here and pass
        // the row pointer blindly into the EvaluateResidualBlock function it will not fill out the row but actually two
        // column elements! That is the reason why we have to specifically specify RowMajor here!
        Eigen::Array<double, Eigen::Dynamic, 2, Eigen::RowMajor> residuals_i{pixels.rows(), 2};

        for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
            ceres::CostFunction const* const cost_function{
                cost_functions::Create(sensor.camera_model, sensor.bounds, pixels.row(i), points.row(i))};

            cost_function->Evaluate(parameter_blocks.data(), residuals_i.row(i).data(), nullptr);

            // TODO(Jack): Should we use a smart pointer instead?
            delete cost_function;
        }

        residuals.insert({timestamp_ns, residuals_i});
    }

    return residuals;
}  // LCOV_EXCL_LINE

// LOCATION!
BaProblem BuildSingleCamBaProblem(CameraInfo const& camera_info, CameraState const& intrinsics, Frames const& frames,
                                  CameraMeasurements const& targets, bool const optimize_intrinsic) {
    // For a single camera problem we do not consider the rig-co extrinsics and set those to constant identity.
    BaCamera const camera{camera_info, BaCameraState{intrinsics, Array6d::Zero()},
                          BaCameraOptions{optimize_intrinsic, false}};

    // Dummy id used for internal problem consistency.
    AssetId const camera_id{0};

    // TODO(Jack): Should we do any check that the frame times match all the target times? Or is that something we need
    // to just check once when we actually construct the problem?
    std::vector<BaObservation> observations;
    for (auto const& [timestamp_ns, target] : targets) {
        observations.push_back({camera_id, timestamp_ns, target.bundle});
    }

    return {{{camera_id, camera}}, frames, observations};
}

}  // namespace  reprojection::optimization
