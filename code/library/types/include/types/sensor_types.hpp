#pragma once

#include <opencv2/opencv.hpp>

#include "types/algorithm_types.hpp"

// NOTE(Jack): We use the word "sensor" to indicate the idea that at this point we are making the relationship between
// data and a specific named data source and world timestamp concrete. These types should, in the best case, stay close
// to the application logic periphery and not penetrate deep into the core code base.

namespace reprojection {

template <typename T>
using StampedData = std::pair<std::uint64_t, T>;

// TODO(Jack): Use concept to check that the first type is the proper expected stamp type.
template <typename T>
using StampedMap = std::map<typename T::first_type, typename T::second_type>;

struct CameraMeasurement {
    uint64_t timestamp_ns;
    ExtractedTarget target;
};
using CameraMeasurements = std::vector<CameraMeasurement>;

using ImuMeasurement = StampedData<ImuData>;
using ImuMeasurements = StampedMap<ImuMeasurement>;

using CameraImage = StampedData<cv::Mat>;

}  // namespace reprojection