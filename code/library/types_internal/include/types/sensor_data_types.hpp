#pragma once

#include <opencv2/opencv.hpp>

#include "types/algorithm_types.hpp"
#include "types/stamped_templates.hpp"

namespace reprojection {

using TargetSample = StampedData<ExtractedTarget>;
using TargetSamples = StampedMap<TargetSample>;

using ImuSample = StampedData<ImuData>;
using ImuSamples = StampedMap<ImuSample>;

// TODO(Jack): We should consider making the absent of an image explicit with std::optional.
using ImageSample = StampedData<ImageBuffer>;
using ImageSamples = StampedMap<ImageSample>;

}  // namespace reprojection