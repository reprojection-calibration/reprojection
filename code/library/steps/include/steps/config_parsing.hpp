#pragma once

#include "config/config_parse.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

// TODO UPDATE
// NOTE(Jack): The entity holds a special place in the calibration process in that it is not part of the regular
// cachable step workflow process. That is readily apparent by the foreign key dependency of the calibration step
// table on the entity ids found in the entity table. The most important implication of this is that the entity
// table needs a custom read/write step like object. And in addition to that, instead of pure "insert" semantics
// the entity use "insert or ignore" semantics because every time the calibration runs it will try to insert
// all the entities that are present. And if it is already present that is not an error!
config::Config ConfigParsing(toml::table const& cfg_table, SqlitePtr const db);

}  // namespace reprojection::steps
