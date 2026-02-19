#pragma once

#include <opencv2/opencv.hpp>

#include "types/algorithm_types.hpp"

// NOTE(Jack): We use the word "sensor" to indicate the idea that at this point we are making the relationship between
// data and a specific named data source and world timestamp concrete. These types should, in the best case, stay close
// to the application logic periphery and not penetrate deep into the core code base.

namespace reprojection {

struct CameraMeasurement {
    uint64_t timestamp_ns;
    ExtractedTarget target;
};

struct ImuMeasurement {
    uint64_t timestamp_ns;
    Vector3d angular_velocity;
    Vector3d linear_acceleration;
};

}  // namespace reprojection