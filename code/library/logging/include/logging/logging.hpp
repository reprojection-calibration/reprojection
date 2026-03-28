#pragma once

#include <spdlog/spdlog.h>

#include <toml++/toml.hpp>

namespace reprojection::logging {

std::shared_ptr<spdlog::logger> Get(std::string const& name);

std::string ToOneLineJson(toml::table const& tbl);

}  // namespace reprojection::logging
