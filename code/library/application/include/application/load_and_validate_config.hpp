#pragma once

#include <variant>

#include <toml++/toml.hpp>

#include "config/types.hpp"

namespace reprojection::application {

std::variant<config::ParserErrorMsg, toml::table> LoadAndValidateConfig();

}
