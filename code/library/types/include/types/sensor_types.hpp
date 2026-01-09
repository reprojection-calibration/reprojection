#pragma once

#include <opencv2/opencv.hpp>

#include "types/algorithm_types.hpp"
#include "types/eigen_types.hpp"

// NOTE(Jack): We use the word "sensor" to indicate the idea that at this point we are making the relationship between
// data and a specific named data source and world timestamp concrete. These types should, in the best case, stay close
// to the application logic periphery and not penetrate deep into the core code base.

namespace reprojection {

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

struct ImuStamped {
    FrameHeader header;
    // TODO(Jack): In future should we refactor the angular_velocity and linear_acceleration into an Imu struct?
    double angular_velocity[3];
    double linear_acceleration[3];

    bool operator<(ImuStamped const& other) const { return header.timestamp_ns < other.header.timestamp_ns; }
};

struct ImageStamped {
    FrameHeader header;
    cv::Mat image;
};

struct PoseStamped {
    FrameHeader header;
    Vector6d pose;
};

}  // namespace reprojection