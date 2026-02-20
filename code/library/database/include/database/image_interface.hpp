#pragma once

#include <memory>
#include <optional>
#include <string>

#include <opencv2/opencv.hpp>

#include "database/calibration_database.hpp"
#include "database/sqlite_wrappers.hpp"
#include "types/sensor_types.hpp"

// NOTE(Jack): We need this streaming interface here because it is not feasible to load all images at once into memory,
// we will run into problems here with memory. Therefore, we create this streamer class which loads the images one by
// one from the database. We include the start_time parameter so that applications can skip loading images prior to that
// timestamp (i.e. if the images before start_time were already processed).
// NOTE(Jack): We put the image interface in its own file because the image streamer interface/concept is so different
// from the other data interfaces that it deserves some separation. The basic idea is that we should not load all images
// at once, but rather "stream" them one at a time. If we try to load all images at once we can easily blow up the
// entire memory of a computer if the dataset is large enough.
// WARN(Jack): The image and extracted target sensor/database interface are unique because those are the only parts
// which will likely be used by external parties. Most other database interfaces are used exclusively by the core
// calibration algorithm. Maybe for external facing components it would be nice to provide an interface with bool or
// std::optional instead of void+throw. Think about this over time!

namespace reprojection::database {

void AddImage(CameraImage const& data, std::string_view sensor_name,
              std::shared_ptr<CalibrationDatabase> const database);

// TODO(Jack): Should we make this instead accept OptimizationState directly and iterate over that?
void AddImage(uint64_t const timestamp_ns, std::string_view sensor_name,
              std::shared_ptr<CalibrationDatabase> const database);

class ImageStreamer {
   public:
    ImageStreamer(std::shared_ptr<CalibrationDatabase const> const database, std::string const& sensor_name,
                  uint64_t const start_time = 0);

    std::optional<CameraImage> Next();

   private:
    std::shared_ptr<CalibrationDatabase const> database_;
    SqlStatement statement_;
    std::string sensor_name_;
};

}  // namespace reprojection::database