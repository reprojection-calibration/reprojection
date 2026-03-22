#include "config/config_parsing.hpp"

namespace reprojection::config {

template <typename T>
T ExtractValue(std::string_view key, toml::table& cfg) {
    T value;
    if (auto const node{cfg.get(key)}) {
        value = node->as<T>()->get();
        cfg.erase(key);
    }

    return value;
}  // LCOV_EXCL_LINE

void ThrowIfUnexpectedKeys(toml::table const& cfg, std::string_view section) {
    if (cfg.empty()) {
        return;
    }

    std::ostringstream oss;
    oss << "Unexpected parameters found in the " << section << " configuration, are you sure they are correct?\n";
    for (const auto& [key, _] : cfg) {
        oss << "  - " << key.str() << "\n";
    }

    throw std::runtime_error(oss.str());
}

// TODO USE FILESYSTEM PATH?
std::string ParseDataConfig(toml::table data_cfg) {
    std::string const file{ExtractValue<std::string>("file", data_cfg)};

    ThrowIfUnexpectedKeys(data_cfg, "data");

    return file;
}

std::pair<std::string, CameraModel> ParseSensorConfig(toml::table sensor_cfg) {
    std::string camera_name{ExtractValue<std::string>("camera_name", sensor_cfg)};
    std::string camera_model{ExtractValue<std::string>("camera_model", sensor_cfg)};

    ThrowIfUnexpectedKeys(sensor_cfg, "sensor");

    return {camera_name, ToCameraModel(camera_model)};
}

}  // namespace reprojection::config