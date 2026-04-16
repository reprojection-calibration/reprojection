#pragma once

#include "database/calibration_database.hpp"
#include "types/sensor_data_types.hpp"

// TODO(Jack): Now that removed the image streamer is there any reason for these to be in their own file? Or exist at
// all?

namespace reprojection::database {

void AddImage(Image const& data, std::string_view sensor_name, SqlitePtr const db);

// TODO(Jack): Should we make this instead accept OptimizationState directly and iterate over that?
void AddImage(uint64_t const timestamp_ns, std::string_view sensor_name, SqlitePtr const db);

}  // namespace reprojection::database