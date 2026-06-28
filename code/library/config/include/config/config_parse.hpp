#pragma once

#include <array>
#include <string>
#include <variant>

#include <toml++/toml.hpp>

#include "config/enums.hpp"
#include "types/config.hpp"
#include "types/enums.hpp"

namespace reprojection::config {

struct Config {
    static std::optional<Config> Parse(toml::table cfg);

    struct Application {
        static std::variant<Application, TomlErrorMsg> Parse(toml::table cfg);

        static ConfigTable TableType() { return ConfigTable::Application; }

        bool show_extraction;
        int threads;
    };

    struct Camera {
        static std::variant<Camera, TomlErrorMsg> Parse(toml::table cfg);

        static ConfigTable TableType() { return ConfigTable::Camera; }

        std::string sensor_name;
        CameraModel camera_model;
    };

    struct Imu {
        static std::variant<Imu, TomlErrorMsg> Parse(toml::table cfg);

        static ConfigTable TableType() { return ConfigTable::Imu; }

        std::string sensor_name;
    };

    struct Target {
        static std::variant<Target, TomlErrorMsg> Parse(toml::table cfg);

        static ConfigTable TableType() { return ConfigTable::Target; }

        TargetType target_type;
        std::array<int, 2> size;
        double unit_dimension;
        bool asymmetric;
    };

   private:
    Config(Application const& app, Camera const& camera, std::optional<Imu> const& imu, Target const& target);

   public:
    Application app;
    Camera camera;
    std::optional<Imu> imu;
    Target target;
};

}  // namespace reprojection::config