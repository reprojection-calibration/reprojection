#include "config/config_parsing.hpp"

#include "parsing_helpers.hpp"

namespace reprojection::config {

std::pair<std::string, CameraModel> ParseSensorConfig(toml::table sensor_cfg) {
    auto const sensor_name{ExtractValue<std::string>("sensor_name", sensor_cfg)};
    auto const camera_model{ExtractValue<std::string>("camera_model", sensor_cfg)};

    // TODO(Jack): Now that we are essentially doing validity checking here what purpose does the "config validation"
    // serve for us now? Are we sure we still need both or can we just do it here directly? I think for now it makes
    // sense to keep them seperate and double check, but that might change. Not a pressing issue for now regardless :)
    if (not sensor_name or not camera_model) {
        throw std::runtime_error{std::format("Error during sensor config parse: sensor_name = {}, camera_model = {}",
                                             camera_model ? *camera_model : "N/A", sensor_name ? *sensor_name : "N/A")};
    }

    ThrowIfUnexpectedKeys(sensor_cfg, "sensor");

    return {*sensor_name, ToCameraModel(*camera_model)};
}

TargetInfo ParseTargetConfig(toml::table target_cfg) {
    auto const type{ExtractValue<std::string>("type", target_cfg)};
    auto const pattern_size{ExtractArray<int, 2>("pattern_size", target_cfg)};

    if (not type or not pattern_size) {
        throw std::runtime_error{std::format("Error during target config parse: type = {}, height = {}, width = {}",
                                             type ? *type : "N/A",
                                             pattern_size ? std::to_string((*pattern_size)[0]) : "N/A",
                                             pattern_size ? std::to_string((*pattern_size)[1]) : "N/A")};
    }

    double unit_dimension{1};
    if (auto const unit_dimension_parse{ExtractValue<double>("unit_dimension", target_cfg)}) {
        unit_dimension = *unit_dimension_parse;
    }

    // TODO(Jack): It is turtles all the way down. As we do not have a principled pattern to deal with related nested
    // tables at this scope we just write this directly here by hand. I do not think this case of nested related tables
    // will come up to often, but if it does we should implement a good pattern.
    bool asymmetric{false};
    if (auto circle_grid_cfg{ExtractTable("circle_grid", target_cfg)}) {
        if (auto const asymmetric_parse{ExtractValue<bool>("asymmetric", *circle_grid_cfg)}) {
            asymmetric = *asymmetric_parse;
        }

        ThrowIfUnexpectedKeys(*circle_grid_cfg, "target.circle_grid");
    }

    ThrowIfUnexpectedKeys(target_cfg, "target");

    return TargetInfo{ToTargetType(*type), (*pattern_size)[0], (*pattern_size)[1], unit_dimension, asymmetric};
}

}  // namespace reprojection::config