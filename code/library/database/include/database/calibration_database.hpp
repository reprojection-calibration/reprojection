#pragma once

#include <memory>
#include <string>

#include "types/io.hpp"

namespace reprojection::database {

SqlitePtr OpenCalibrationDatabase(std::string const& db_path, bool const create, bool const read_only = false);

}  // namespace reprojection::database