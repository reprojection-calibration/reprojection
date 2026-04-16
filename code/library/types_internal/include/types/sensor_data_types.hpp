#pragma once

#include <opencv2/opencv.hpp>

#include "types/algorithm_types.hpp"
#include "types/stamped_templates.hpp"

namespace reprojection {

using CameraMeasurement = StampedData<ExtractedTarget>;
using CameraMeasurements = StampedMap<CameraMeasurement>;

using ImuMeasurement = StampedData<ImuData>;
using ImuMeasurements = StampedMap<ImuMeasurement>;

using Image = StampedData<cv::Mat>;

// TODO(Jack): We should consider making the absent of an image explicit with std::optional.
using EncodedImage = StampedData<ImageBuffer>;
using EncodedImages = StampedMap<EncodedImage>;

}  // namespace reprojection