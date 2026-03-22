#include "config/config_parsing.hpp"

namespace reprojection::config {

// TODO USE FILESYSTEM PATH?
std::string ParseDataConfig(toml::table data_cfg) {
    // TODO MAKE FUNCTION TO ELIMINATE COPY PASTE
    std::string file;
    if (auto const value{data_cfg.get("file")}) {
        file = value->as<std::string>()->get();
        data_cfg.erase("file");
    }


    // TODO MAKE FUNCTION TO ELIMINATE COPY PASTE
    if (not data_cfg.empty()) {
        std::ostringstream oss;
        oss << "Unexpected parameters found in the sensor configuration, are you sure they are correct?\n";
        for (const auto& [key, _] : data_cfg) {
            oss << "  - " << key.str() << "\n";
        }

        throw std::runtime_error(oss.str());
    }

    return file;
}

std::pair<std::string, CameraModel> ParseSensorConfig(toml::table sensor_cfg) {
    // TODO MAKE FUNCTION TO ELIMINATE COPY PASTE
    std::string camera_name;
    if (auto const value{sensor_cfg.get("camera_name")}) {
        camera_name = value->as<std::string>()->get();
        sensor_cfg.erase("camera_name");
    }

    // TODO MAKE FUNCTION TO ELIMINATE COPY PASTE
    std::string camera_model;
    if (auto const value{sensor_cfg.get("camera_model")}) {
        camera_model = value->as<std::string>()->get();
        sensor_cfg.erase("camera_model");
    }

    // TODO MAKE FUNCTION TO ELIMINATE COPY PASTE
    if (not sensor_cfg.empty()) {
        std::ostringstream oss;
        oss << "Unexpected parameters found in the sensor configuration, are you sure they are correct?\n";
        for (const auto& [key, _] : sensor_cfg) {
            oss << "  - " << key.str() << "\n";
        }

        throw std::runtime_error(oss.str());
    }

    return {camera_name, ToCameraModel(camera_model)};
}

}  // namespace reprojection::config