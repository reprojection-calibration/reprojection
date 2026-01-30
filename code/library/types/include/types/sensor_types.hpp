#pragma once

#include <opencv2/opencv.hpp>

#include "types/algorithm_types.hpp"

// NOTE(Jack): We use the word "sensor" to indicate the idea that at this point we are making the relationship between
// data and a specific named data source and world timestamp concrete. These types should, in the best case, stay close
// to the application logic periphery and not penetrate deep into the core code base.

namespace reprojection {

// TODO NO LONGER USED?
// TODO(Jack): Add concept requirements
// TODO(Jack): Where does this belong?
template <typename T>
bool operator<(T const& x, T const& y) {
    return x.header.timestamp_ns < y.header.timestamp_ns;
}

struct FrameHeader {
    uint64_t timestamp_ns;
    std::string sensor_name;
};

struct ExtractedTargetStamped {
    FrameHeader header;
    ExtractedTarget target;
};

struct ImuMeasurement {
    Vector3d angular_velocity;
    Vector3d linear_acceleration;
};

struct ImuStamped {
    FrameHeader header;
    ImuMeasurement data;
};

struct ImageStamped {
    FrameHeader header;
    cv::Mat image;
};

}  // namespace reprojection