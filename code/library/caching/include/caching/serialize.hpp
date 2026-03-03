
#include <string>

#include "types/sensor_data_types.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::caching {

std::string Serialize(CameraInfo const& data);

std::string Serialize(CameraMeasurements const& data);

}  // namespace reprojection::caching
