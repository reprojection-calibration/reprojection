#pragma once

#include <toml++/toml.hpp>

#include "application/calibration_database_forward_declaration.hpp"
#include "types/io.hpp"

namespace reprojection::application {

// TODO(Jack): Central definition?
using DbPtr = std::shared_ptr<database::CalibrationDatabase>;

// TODO(Jack): How should we pass the ImageSource?
void Calibrate(toml::table const& config, ImageSource image_source, std::string const& image_source_signature,
               DbPtr const db);

}  // namespace reprojection::application
