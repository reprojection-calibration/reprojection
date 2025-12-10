#pragma once

#include <sqlite3.h>

#include <cstdint>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>
#include <set>
#include <string>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::database {

struct ImuData {
    uint64_t timestamp_ns;
    double angular_velocity[3];
    double linear_acceleration[3];
};

struct ImageData {
    uint64_t timestamp_ns;
    cv::Mat image;
};

bool operator<(ImuData const& x, ImuData const& y) { return x.timestamp_ns < y.timestamp_ns; }

// TODO(Jack): At this time we are going to hardcode the fact that there is only one possible target for any
// calibration, by not adding a target_id to the table. However it might also make sense to attach a target name/id to
// each data here, because it can be that a user would want to use multiple targets and identify the extracted features
// uniquely based on the timestamp, which sensor they belong to, and from which target they come from.
[[nodiscard]] bool AddExtractedTarget(std::string const& sensor_name, ExtractedTarget const& data,
                                      std::shared_ptr<CalibrationDatabase> const database);

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

    ~ImageStreamer();

    std::optional<ImageData> Next();

   private:
    std::shared_ptr<CalibrationDatabase const> database_;
    sqlite3_stmt* stmt_{nullptr};
};

}  // namespace reprojection::database