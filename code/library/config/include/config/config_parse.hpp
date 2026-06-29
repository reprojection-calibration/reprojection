#pragma once

#include <array>
#include <string>
#include <thread>

#include <toml++/toml.hpp>

#include "types/enums.hpp"

namespace reprojection::config {

struct Config {
    static Config Parse(toml::table const& table);

    struct Application {
        static Application Parse(toml::table const& table);

        bool show_extraction{false};
        int threads{std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 1)};
    };

    struct Camera {
        static Camera Parse(toml::table const& table);

        std::string sensor_name;
        CameraModel camera_model;
    };

    struct Imu {
        static std::optional<Imu> Parse(toml::table const& table);

        std::string sensor_name;
    };

    struct Target {
        static Target Parse(toml::table const& table);

        TargetType target_type;
        std::array<int, 2> size;
        double unit_dimension{1.0};
        bool asymmetric{false};
    };

    Application application;
    Camera camera;
    std::optional<Imu> imu;
    Target target;
};

}  // namespace reprojection::config