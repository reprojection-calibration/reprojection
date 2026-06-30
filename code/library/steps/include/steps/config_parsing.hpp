#pragma once

#include "config/config_parse.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

// WARN(Jack): Config parsing is not a normal step! It holds the unique position of initializing the "entities". Because
// of this it cannot be in the calibration_steps table because that table itself has a foreign key dependency on the
// entities. It is also unique because as of now the config file is required to always be present which means that there
// is no real reason/logical need to cache it here.  This piece of code is the root of the tree and does not fit well
// into our current step paradigm. Time will tell if this was the right choice or not. It would be cool to have this be
// a step like all others, but I can't think of how to do it.
config::Config ConfigParsing(toml::table const& cfg_table, SqlitePtr const db);

}  // namespace reprojection::steps
