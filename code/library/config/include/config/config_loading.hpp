#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <variant>

#include <toml++/toml.hpp>

#include "types/config.hpp"
#include "types/enums.hpp"

namespace reprojection::config {

struct CalibrationConfig {
    static CalibrationConfig Load(std::filesystem::path const& path);

   private:
    CalibrationConfig() = default;

   public:
    struct Application {
        static Application Load(toml::table cfg);

        bool show_extraction;
        int threads;
    };

    struct Camera {
        static Camera Load(toml::table cfg);

        std::string sensor_name;
        CameraModel camera_model;
    };

    struct Imu {
        static Imu Load(toml::table cfg);

        std::string sensor_name;
    };

    struct Target {
        static Target Load(toml::table cfg);

        std::array<int, 2> size;
        TargetType target_type;
    };

    Workflow Workflow() const { return imu.has_value() ? Workflow::CameraImu : Workflow::Camera; }

    Application app;
    Camera camera;
    std::optional<Imu> imu;
    Target target;
};

std::variant<toml::table, TomlErrorMsg> LoadConfigFile(std::string const& file);

}  // namespace reprojection::config