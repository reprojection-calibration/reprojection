#include <string>

#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::caching {

std::string CacheKey(CameraInfo const& sensor, CameraMeasurements const& targets, CameraState const& intrinsics);

}  // namespace reprojection::caching
