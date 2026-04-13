#pragma once

#include <toml++/toml.hpp>

#include "types/io.hpp"

namespace reprojection::application {

// TODO(Jack): How should we pass the ImageSource?
void Calibrate(toml::table const& config, ImageSource image_source, std::string const& image_source_signature,
               SqlitePtr const db);

}  // namespace reprojection::application
