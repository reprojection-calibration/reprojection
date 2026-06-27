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

        TargetType target_type;
        std::array<int, 2> size;
        double unit_dimension;
        bool asymmetric;
    };

    Workflow QueryWorkflow() const { return imu.has_value() ? Workflow::CameraImu : Workflow::Camera; }

   private:
    Config(Application const& app, Camera const& camera, std::optional<Imu> const& imu, Target const& target);

   public:
    Application app;
    Camera camera;
    std::optional<Imu> imu;
    Target target;
};

}  // namespace reprojection::config