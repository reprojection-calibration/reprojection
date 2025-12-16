#pragma once

#include <sqlite3.h>

#include <cstdint>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>
#include <set>
#include <string>

#include "database/calibration_database.hpp"
#include "database/database_data_types.hpp"


namespace reprojection::database {

struct SqlStatement {
    SqlStatement(sqlite3* const db, char const* const sql);

    ~SqlStatement();

    sqlite3_stmt* stmt{nullptr};
};

// TODO(Jack): The pose topic here is not well fleshed out in the sql idea so far! You cannot pass the table name
// dynamically which means, given our current code generation in the cmakelists, that just need to make a set of pose
// statements for every pose table we want to have. Considering that the only difference between them all will be the
// name of the table, this is a bad case of code duplication! We need to find a way to solve this by parameterizing the
// table name somehow, possibly just at run time.
[[nodiscard]] bool AddCameraPoseData(std::string const& sensor_name, PoseData const& data, PoseType const type,
                                     std::shared_ptr<CalibrationDatabase> const database);

[[nodiscard]] bool AddCameraPoseData(std::string const& sensor_name, std::set<PoseData> const& data,
                                     PoseType const type, std::shared_ptr<CalibrationDatabase> const database);

// TODO(Jack): At this time we are going to hardcode the fact that there is only one possible target for any
// calibration, by not adding a target_id to the table. However it might also make sense to attach a target name/id to
// each data here, because it can be that a user would want to use multiple targets and identify the extracted features
// uniquely based on the timestamp, which sensor they belong to, and from which target they come from.
[[nodiscard]] bool AddExtractedTargetData(std::string const& sensor_name, ExtractedTargetData const& data,
                                          std::shared_ptr<CalibrationDatabase> const database);

// WARN(Jack): Assumes only one single target
std::optional<std::set<ExtractedTargetData>> GetExtractedTargetData(
    std::shared_ptr<CalibrationDatabase const> const database, std::string const& sensor_name);

[[nodiscard]] bool AddImuData(std::string const& sensor_name, ImuData const& data,
                              std::shared_ptr<CalibrationDatabase> const database);

std::optional<std::set<ImuData>> GetImuData(std::shared_ptr<CalibrationDatabase const> const database,
                                            std::string const& sensor_name);

// TODO(Jack): Code protections that only grayscale images are stored here? Or is this not the right place?
// NOTE(Jack): There are so many different error conditions that we definsively program against, but that we cannot
// reasonably test against, so we have to use LCOV_EXCL_LINE a lot in the database image handling :(
[[nodiscard]] bool AddImage(std::string const& sensor_name, ImageData const& data,
                            std::shared_ptr<CalibrationDatabase> const database);

// NOTE(Jack): We need this streaming interface here because it is not feasible to load all images at once into memory,
// we will run into problems here with memory. Therefore, we create this streamer class which loads the images one by
// one from the database. We include the start_time parameter so that applications can skip loading images prior to that
// timestamp (i.e. if the images before start_time were already processed).
// WARN(Jack): We need to codify how we deal with const with the database. At this point I think there is no clear
// strategy or we might think we have protection where we do not really.
class ImageStreamer {
   public:
    ImageStreamer(std::shared_ptr<CalibrationDatabase const> const database, std::string const& sensor_name,
                  uint64_t const start_time = 0);

    std::optional<ImageData> Next();

   private:
    std::shared_ptr<CalibrationDatabase const> database_;
    SqlStatement statement_;
};

}  // namespace reprojection::database