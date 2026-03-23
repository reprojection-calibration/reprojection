#pragma once

#include <opencv2/opencv.hpp>

#include "types/algorithm_types.hpp"
#include "types/stamped_templates.hpp"

namespace reprojection {

using CameraMeasurement = StampedData<ExtractedTarget>;
using CameraMeasurements = StampedMap<CameraMeasurement>;

using ImuMeasurement = StampedData<ImuData>;
using ImuMeasurements = StampedMap<ImuMeasurement>;

using CameraImage = StampedData<cv::Mat>;

}  // namespace reprojection