#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <variant>

#include <toml++/toml.hpp>

#include "types/config.hpp"
#include "types/enums.hpp"

namespace reprojection::config {

struct Config {
    static std::optional<Config> Load(std::filesystem::path const& path);

   private:
    Config() = default;

   public:
    struct Application {
        static std::variant<Application, TomlErrorMsg> Parse(toml::table cfg);

        bool show_extraction;
        int threads;
    };

    struct Camera {
        static std::variant<Camera, TomlErrorMsg> Parse(toml::table cfg);

        std::string sensor_name;
        CameraModel camera_model;
    };

    struct Imu {
        static std::variant<Imu, TomlErrorMsg> Parse(toml::table cfg);

        std::string sensor_name;
    };

    struct Target {
        static std::variant<Target, TomlErrorMsg> Parse(toml::table cfg);

        std::array<int, 2> size;
        TargetType target_type;
    };

    Workflow QueryWorkflow() const { return imu.has_value() ? Workflow::CameraImu : Workflow::Camera; }

    Application app;
    Camera camera;
    std::optional<Imu> imu;
    Target target;
};

}  // namespace reprojection::config