#pragma once

#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

// NOTE(Jack): This file really exists for datatypes that have an associated timestamp. This is the first place where we
// are encountering the notion of a global world time assocaited with sensor measurements, so the naming or entire
// concept can change here quickly if needed;

namespace reprojection::database {

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
};

struct ImageStamped {
    FrameHeader header;
    cv::Mat image;
};

struct PoseStamped {
    uint64_t timestamp_ns;
    Vector6d pose;
};

enum class PoseType { Initial, Optimized };

// TODO(Jack): Where does this belong?
inline std::string ToString(PoseType const t) {
    switch (t) {
        case PoseType::Initial:
            return "initial";
        case PoseType::Optimized:
            return "optimized";
    }
    throw std::logic_error("Invalid PoseType");
}

// TODO(Jack): Add concept requirements
// TODO(Jack): Where does this belong?
// TODO REMOVE SECOND
template <typename T>
bool operator<(T const& x, T const& y) {
    return x.timestamp_ns < y.timestamp_ns;
}

inline bool operator<(ExtractedTargetStamped const& x, ExtractedTargetStamped const& y) {
    return x.header.timestamp_ns < y.header.timestamp_ns;
}

inline bool operator<(ImuStamped const& x, ImuStamped const& y) {
    return x.header.timestamp_ns < y.header.timestamp_ns;
}

}  // namespace reprojection::database