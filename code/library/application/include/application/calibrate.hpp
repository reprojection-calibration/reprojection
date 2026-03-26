#pragma once

#include "types/calibration_types.hpp"

#include <toml++/toml.hpp>

// TODO NOTE WE ARE INCLUDING THIS FOR THE FORWARD DEFINITION OF TEH DATABASE! MAYBE WE SHOULD PUT THAT IN ITS OWN
// HEADER?
#include "application/database.hpp"

namespace reprojection::application {

// TODO(Jack): Should we make a central definition of this? COPY AND PASTED
using ImageProvider = std::function<std::optional<std::pair<uint64_t, cv::Mat>>()>;

void Calibrate(toml::table const& config, ImageProvider image_source, std::string_view image_cache_key, DbPtr const db);

}  // namespace reprojection::application
