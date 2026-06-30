#pragma once

#include "config/config_parse.hpp"

namespace reprojection::steps {

config::Config ConfigParsing(toml::table const& cfg_table, SqlitePtr const db);

}
