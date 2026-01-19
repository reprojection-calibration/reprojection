#pragma once

#include <optional>
#include <variant>

#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

// https://rpg.ifi.uzh.ch/docs/teaching/2020/03_camera_calibration.pdf

namespace reprojection::pnp {

enum class PnpErrorCode {
    ContainsNan,
    FailedRefinement,
    InvalidDlt,
};

// TODO(Jack): Is it bad to use a using declaration here in the public API section?
using PnpResult = std::variant<Isometry3d, PnpErrorCode>;

PnpResult Pnp(Bundle const& bundle, std::optional<ImageBounds> bounds = std::nullopt);

}  // namespace reprojection::pnp