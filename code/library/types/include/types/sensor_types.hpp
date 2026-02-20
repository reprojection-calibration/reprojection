#pragma once

#include <opencv2/opencv.hpp>

#include "types/algorithm_types.hpp"

namespace reprojection {

// TODO(Jack): Use concept to check that the first type for StampedMap is the proper expected stamp type.
template <typename T>
using StampedData = std::pair<std::uint64_t, T>;
template <typename T>
using StampedMap = std::map<typename T::first_type, typename T::second_type>;

using CameraMeasurement = StampedData<ExtractedTarget>;
using CameraMeasurements = StampedMap<CameraMeasurement>;

using ImuMeasurement = StampedData<ImuData>;
using ImuMeasurements = StampedMap<ImuMeasurement>;

using CameraImage = StampedData<cv::Mat>;

}  // namespace reprojection