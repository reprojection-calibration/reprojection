#pragma once

#include <variant>

#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

// https://rpg.ifi.uzh.ch/docs/teaching/2020/03_camera_calibration.pdf

namespace reprojection::pnp {

// TODO(Jack): Should this be called an "error" code instead? If it was really status code I would expect that when it
// is succesful we also  get a code, but right now we get the tf instead and no code.
enum class PnpStatusCode {
    MismatchedCorrespondences,
    NotEnoughPoints,
};

// TODO(Jack): Is it bad to use a using declaration here in the public API section?
using PnpResult = std::variant<Isometry3d, PnpStatusCode>;

PnpResult Pnp(Bundle const& bundle);

}  // namespace reprojection::pnp