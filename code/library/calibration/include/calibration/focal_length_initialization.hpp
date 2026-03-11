#pragma once

#include <ranges>

#include "calibration/linear_pose_initialization.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"
#include "projection_functions/double_sphere.hpp"
#include "projection_functions/projection_class_concept.hpp"
#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"
#include "types/enums.hpp"

namespace reprojection::calibration {

std::vector<double> InitializeFocalLengthParabolaLine(ExtractedTarget const& target, Vector2d const& principal_point);

template <typename T_Model>
    requires projection_functions::ProjectionClass<T_Model>
std::optional<double> InitializeIntrinsics(CameraMeasurements const& targets, double const height, double const width) {
    // TODO(Jack):: Add vanishing point init!
    auto const runner{[height, width](ExtractedTarget const& target) {
        if constexpr (T_Model::InitType == InitializationType::ParabolaLine) {
            return InitializeFocalLengthParabolaLine(target, Vector2d{height / 2, width / 2});
        }
    }};

    std::vector<double> gammas;
    for (auto const& target : targets | std::views::values) {
        std::vector<double> const gammas_i{runner(target)};
        gammas.insert(std::end(gammas), std::cbegin(gammas_i), std::cend(gammas_i));
    }

    // TODO(Jack): This is not so nice but we need to detect which camera_model enum type we have to fill out the inputs
    //  required for LPI.
    CameraModel camera_model;
    if constexpr (std::is_same_v<T_Model, projection_functions::DoubleSphere>) {
        camera_model = CameraModel::DoubleSphere;
    } else {
        static_assert(false);
    }

    // TODO (Jack): This is overkill because we calculate the reprojection error across all frames when we should just
    //  use the one frame we got the gamma from.
    std::optional<double> best_gamma;
    double best_error{std::numeric_limits<double>().max()};
    for (auto const gamma : gammas) {
        // TODO(Jack): Are width and height in the right order?
        CameraInfo const camera_info{"", camera_model, ImageBounds{0, width, 0, height}};
        CameraState const camera_state{T_Model::Initialize(gamma, height, width)};
        Frames const frames{LinearPoseInitialization(camera_info, targets, camera_state)};

        OptimizationState const full_state{camera_state, frames};
        ReprojectionErrors const errors{optimization::ReprojectionResiduals(camera_info, targets, full_state)};

        double error_sum{0};
        for (auto const& error_i : errors | std::views::values) {
            error_sum += error_i.array().abs().mean();
        }
        if (error_sum < best_error) {
            best_gamma = gamma;
            best_error = error_sum;
        }
    }

    return best_gamma;
}

}  // namespace reprojection::calibration
