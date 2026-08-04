#pragma once

#include <toml++/toml.hpp>

#include "types/eigen_types.hpp"
#include "types/enums.hpp"

namespace reprojection::database {

std::string ToToml(CameraModel type, ArrayXd const& intrinsics);

ArrayXd FromToml(CameraModel type, std::string const& json_str);

ArrayXd ReadPinholeValues(toml::parse_result const& tbl, ArrayXd intrinsics);

}  // namespace reprojection::database