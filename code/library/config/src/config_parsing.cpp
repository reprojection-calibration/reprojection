#include "config/config_parsing.hpp"

#include "parsing_helpers.hpp"

namespace reprojection::config {

std::pair<std::string, CameraModel> ParseSensorConfig(toml::table sensor_cfg) {
    auto const camera_name{ExtractValue<std::string>("camera_name", sensor_cfg)};
    auto const camera_model{ExtractValue<std::string>("camera_model", sensor_cfg)};

    // TODO(Jack): Now that we are essentially doing validity checking here what purpose does the "config validation"
    // serve for us now? Are we sure we still need both or can we just do it here directly? I think for now it makes
    // sense to keep them seperate and double check, but that might change. Not a pressing issue for now regardless :)
    if (not camera_name or not camera_model) {
        throw std::runtime_error{std::format("Error during sensor config parse: camera_name = {}, camera_model = {}",
                                             camera_model ? *camera_model : "N/A", camera_name ? *camera_name : "N/A")};
    }

    ThrowIfUnexpectedKeys(sensor_cfg, "sensor");

    return {*camera_name, ToCameraModel(*camera_model)};
}

TargetInfo ParseTargetConfig(toml::table target_cfg) {
    auto const type{ExtractValue<std::string>("type", target_cfg)};
    auto const height{ExtractValue<int64_t>("height", target_cfg)};
    auto const width{ExtractValue<int64_t>("width", target_cfg)};

    if (not type or not height or not width) {
        throw std::runtime_error{std::format("Error during target config parse: type = {}, height = {}, width = {}",
                                             type ? *type : "N/A", height ? std::to_string(*height) : "N/A",
                                             width ? std::to_string(*width) : "N/A")};
    }

    // ADD UNIT DIMENSION TO TARGET INFO!!!!
    // ADD UNIT DIMENSION TO TARGET INFO!!!!
    // ADD UNIT DIMENSION TO TARGET INFO!!!!
    // ADD UNIT DIMENSION TO TARGET INFO!!!!
    // auto const unit_dimension{ExtractValue<double>("unit_dimension", target_cfg)};

    auto const asymmetric{ExtractValue<bool>("circle_grid.asymmetric", target_cfg)};

    ThrowIfUnexpectedKeys(target_cfg, "target");

    return TargetInfo{ToTargetType(*type), static_cast<int>(*height), static_cast<int>(*width), *asymmetric};
}

}  // namespace reprojection::config