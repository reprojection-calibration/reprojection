#pragma once

#include <ranges>

#include "projection_functions/projection_class_concept.hpp"
#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"
#include "types/enums.hpp"

namespace reprojection::calibration {

std::vector<double> InitializeFocalLengthParabolaLine(ExtractedTarget const& target, Vector2d const& principal_point);

template <typename T_Model>
    requires projection_functions::ProjectionClass<T_Model>
std::vector<double> InitializeIntrinsics(CameraMeasurements const& targets, double const height, double const width) {
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

    // TODO CALCULATE REPROJECTION ERROR

    // TODO RETURN SINGLE INTRINSIC ARRAY!
    return gammas;
}

}  // namespace reprojection::calibration
