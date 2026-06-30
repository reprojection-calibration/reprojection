#pragma once

#include <array>
#include <string>
#include <thread>

#include <toml++/toml.hpp>

#include "types/enums.hpp"

namespace reprojection::config {

// TODO(Jack): Make the actual config datatype independent of the parsing methods. It looks nice to have them together
// like this, but I do not think it is required. One downside of the current setup is that this is found in the config::
// namespace which makes using it more verbose. It also has a dependency on the toml package which is then transiently
// forces on anyone that needs the config, even if they just want the type and not the parsing.

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

    // NOTE(Jack): At a high level there are three kinds of config "requirements"
    //
    //  1) required
    //  2) optional with default value
    //  3) optional with no default value
    //
    // This can apply both to the top level configs below and the individual keys within the configs.
    //
    // Required configs are obvious, those are the things that the program cannot run without, for example the name of
    // the camera sensor or the type of target used. Optional with default value configs are those for which we can set
    // a sensible default value; for example the number of threads used by the optimizer can be parameterized if the
    // user wants, but we can also automatically set a value if they don't care.
    //
    // Optional with no default value are the rarest type and are unique because they actually control the pipeline
    // execution; the only one at time of writing (30.06.2026) is Config::Imu. If the IMU table is in the config file
    // then that is the same as the user telling the pipeline to do a extrinsic calibration. If it is not present in the
    // config file then the user is only asking for a camera intrinsic calibration. Whether or not this is easy for the
    // user to understand, only time will tell :)
    //
    // Look at the config_parse.test.cpp for hands-on examples of what valid configs are and are not.
    Application application;
    Camera camera;
    std::optional<Imu> imu;
    Target target;
};

}  // namespace reprojection::config