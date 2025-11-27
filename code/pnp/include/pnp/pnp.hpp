#pragma once

#include <variant>

#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

// https://rpg.ifi.uzh.ch/docs/teaching/2020/03_camera_calibration.pdf

namespace reprojection::pnp {

// TODO(Jack): Should this be called an "error" code instead? If it was really status code I would expect that when it
// is succesful we also  get a code, but right now we get the tf instead and no code.
// NOTE(Jack): We originally also had the "MismatchedCorrespondence" enum here, but then we required that Bundle always
// has matching row counts, and therefore we designed that error out of existence. For now we leave the idea of
// PnpStatusCode even though it only has one value. If this is useful or not we will see in upcoming work.
enum class PnpStatusCode { NotEnoughPoints };

// TODO(Jack): Naming! Is the only context of place we need a type like this?
struct PnpOutput {
    Isometry3d pose;
    double reprojection_error;
};

// TODO(Jack): Is it bad to use a using declaration here in the public API section?
using PnpResult = std::variant<PnpOutput, PnpStatusCode>;

PnpResult Pnp(Bundle const& bundle);

}  // namespace reprojection::pnp