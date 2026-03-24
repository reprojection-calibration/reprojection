#include "config/config_parsing.hpp"

#include "parsing_helpers.hpp"

namespace reprojection::config {

std::pair<std::string, CameraModel> ParseSensorConfig(toml::table sensor_cfg) {
    std::string const camera_name{ExtractValue<std::string>("camera_name", sensor_cfg)};
    std::string const camera_model{ExtractValue<std::string>("camera_model", sensor_cfg)};

    ThrowIfUnexpectedKeys(sensor_cfg, "sensor");

    return {camera_name, ToCameraModel(camera_model)};
}

}  // namespace reprojection::config