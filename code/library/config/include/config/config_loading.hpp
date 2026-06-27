#pragma once

#include <array>
#include <string>
#include <variant>

#include <toml++/toml.hpp>

#include "types/config.hpp"
#include "types/enums.hpp"

namespace reprojection::config {



struct CalibrationConfig {
    struct Application {
        bool show_extraction;
        int threads;
        Workflow workflow;
    };

    struct Camera {
        std::string sensor_name;
        CameraModel camera_model;
    };

    struct Imu {
        std::string sensor_name;
    };

    struct Target {
        std::array<int, 2> size;
        TargetType target_type;

    };
};

std::variant<toml::table, TomlErrorMsg> LoadConfigFile(std::string const& file);

}  // namespace reprojection::config