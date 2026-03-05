
#include <string>

#include "types/sensor_data_types.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::caching {

std::string Serialize(CameraInfo const& data);

std::string Serialize(CameraMeasurements const& data);

std::string Serialize(CameraState const& data);

std::string Serialize(Frames const& data);

}  // namespace reprojection::caching
