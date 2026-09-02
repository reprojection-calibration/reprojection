#include "optimization/bundle_adjustment.hpp"

#include <ceres/loss_function.h>

#include <ranges>

#include "cost_functions/reprojection_error.hpp"

namespace reprojection::optimization {

// initial_rig_world_tfs (come from the same camera that has an identity rig-camera extrinsic which is kept constant)

// struct {
//     intrinsic_init
//    extracted_targets
//    rig_camera_extrinsic_init
//    optimize_extrinsic_bool
// };

// TODO NAMING!
struct BaCamera {
    struct State {
        CameraState intrinsic;
        // TODO(Jack): Should we use the extrinsic type here?
        Array6d tf_co_rig;
    };

    CameraMeasurements targets;
    CameraInfo camera_info;
    State state;
    bool fixed_intrinsic;
    bool fixed_tf;
};

// THIS ALL NEEDS TO BE SYNCED!!!!

// ERROR(Jack): What is a frame has too few valid pixels to actually constrain the pose? Should we entirely skip
// that frame? Or what if in general we have a minimum required of points per frame threshold?
std::tuple<OptimizationState, CeresState> BundleAdjustment(Frames tf_rig_w, std::vector<BaCamera> const& cameras,
                                                           int const num_threads) {
    CeresState ceres_state{ceres::TAKE_OWNERSHIP, ceres::DENSE_SCHUR};
    ceres_state.solver_options.num_threads = num_threads;
    ceres::Problem problem{ceres_state.problem_options};

    // We are dealing with pointers below so it is important we do not reallocate! Therefore we reserve here.
    std::vector<BaCamera::State> optimizable_state;
    optimizable_state.reserve(std::size(cameras));
    for (auto const& camera : cameras) {
        optimizable_state.push_back(camera.state);
        auto& state_i{optimizable_state.back()};

        for (auto const timestamp_ns : tf_rig_w | std::views::keys) {
            // HOW IS THIS GOING TO WORK WHEN WE HAVE THREE OR MORE CAMERAS? HOW DO WE TIME SYNC THEM ALL?
            if (camera.targets.contains(timestamp_ns) == 0) {
                continue;
            }
            auto const& [pixels, points]{camera.targets.at(timestamp_ns).bundle};

            for (Eigen::Index j{0}; j < pixels.rows(); ++j) {
                ceres::CostFunction* const cost_function{cost_functions::Create(
                    camera.camera_info.camera_model, camera.camera_info.bounds, pixels.row(j), points.row(j))};

                problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0), state_i.intrinsic.intrinsics.data(),
                                         state_i.tf_co_rig.data(), tf_rig_w.at(timestamp_ns).pose.data());

                if (camera.fixed_intrinsic) {
                    problem.SetParameterBlockConstant(state_i.intrinsic.intrinsics.data());
                }
                if (camera.fixed_tf) {
                    problem.SetParameterBlockConstant(state_i.tf_co_rig.data());
                }
            }
        }
    }

    ceres::Solve(ceres_state.solver_options, &problem, &ceres_state.solver_summary);

    return {optimized_state, ceres_state};
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

}  // namespace  reprojection::optimization
