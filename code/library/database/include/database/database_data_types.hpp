#pragma once

#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

// NOTE(Jack): This file really exists for datatypes that have an associated timestamp. This is the first place where we
// are encountering the notion of a global world time assocaited with sensor measurements, so the naming or entire
// concept can change here quickly if needed;

namespace reprojection::database {

struct ExtractedTargetData {
    uint64_t timestamp_ns;
    ExtractedTarget target;
};

struct ImuData {
    uint64_t timestamp_ns;
    double angular_velocity[3];
    double linear_acceleration[3];
};

struct ImageData {
    uint64_t timestamp_ns;
    cv::Mat image;
};

// TODO(Jack): Do better than just appending data to indicate it has a timestamp!
struct PoseData {
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
template <typename T>
bool operator<(T const& x, T const& y) {
    return x.timestamp_ns < y.timestamp_ns;
}
}  // namespace reprojection::database