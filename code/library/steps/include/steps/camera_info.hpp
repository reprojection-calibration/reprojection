#pragma once

#include <toml++/toml.hpp>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

// NOTE(Jack): The CameraInfoStep and FeatureExtractionStep are unique because they need to iterate over the input
// images. Because there are many possible data interfaces with different ways to iterate over them, we chose to use
// dependency injection via a lambda to represent this. That means that any application that wants to call either step
// must provide a lambda which essentially lets us iterate over the images from a single sensor. So far this seems like
// the right abstraction, but I could image that one day this could be made a little more formal because for a new
// engineer figuring out how to implement the lambda could be tricky. But that is future music!
// NOTE(Jack): Also because we want a way to determine if we have a cache hit or not BEFORE we iterate over and
// serialize the entire dataset, we ask for the CameraInfoStep and FeatureExtractionStep that the user passes in a cache
// key/serialized blob that uniquely represents the image data. In ROS for example you can do this by iterating over the
// still serialized messages and summing up the timestamps, or something like that, to create a serialized string which
// uniquely identifies that stream. Depending on each application to do that is not a very strong strategy, and it could
// be that we one day formalize the requirements and cache key construction process for the image data fed from
// applications.

struct CameraInfoStep {
    std::string cache_key;
    // TODO(Jack): We should have structs here not unparsed toml tables, right?
    toml::table sensor_config;
    ImageSource image_source;

    CalibrationStep step_type{CalibrationStep::CameraInfo};

    std::string SensorName() const { return sensor_config["camera_name"].as_string()->get(); }

    std::string CacheKey() const;

    CameraInfo Compute() const;

    CameraInfo Load(std::shared_ptr<database::CalibrationDatabase const> const db) const;

    void Save(CameraInfo const& camera_info, std::shared_ptr<database::CalibrationDatabase> const db) const;
};

}  // namespace reprojection::steps
