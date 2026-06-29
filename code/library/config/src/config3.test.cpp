#include <gtest/gtest.h>

#include <spdlog/fmt/bundled/format.h>  // TODO REMOVE?

#include <algorithm>
#include <thread>

#include <toml++/toml.hpp>

#include "types/enums.hpp"

#include "parsing_helpers3.hpp"

namespace reprojection::config {

// TODO(Jack): Reject unexpected keys!
struct Config {
    struct Application {
        // The table is not required, but we have sensible defaults.
        static Application Parse(toml::table const& table) {
            RejectUnexpectedKeys(table, {"show_extraction", "threads"}, "application");

            Application config{};
            OverrideIfPresent(table, "show_extraction", config.show_extraction);
            OverrideIfPresent(table, "threads", config.threads);

            return config;
        }

        bool show_extraction{false};
        int threads{std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 1)};
    };

    struct Camera {
        static Camera Parse(toml::table const& table) {
            RejectUnexpectedKeys(table, {"sensor_name", "camera_model"}, "camera");

            return Camera{Require<std::string>(table, "sensor_name"),
                          ToCameraModel(Require<std::string>(table, "camera_model"))};
        }

        std::string sensor_name;
        CameraModel camera_model;
    };

    struct Imu {
        // The table is not required (we do not always have IMU data), but we have no sensible defaults.
        static std::optional<Imu> Parse(toml::table const& table) {
            RejectUnexpectedKeys(table, {"sensor_name"}, "imu");

            auto const sensor_name{Optional<std::string>(table, "sensor_name")};
            if (not sensor_name) {
                return std::nullopt;
            }

            return Imu{*sensor_name};
        }

        std::string sensor_name;
    };

    struct Target {
        static Target Parse(toml::table const& table) {
            RejectUnexpectedKeys(table, {"type", "pattern_size", "unit_dimension", "circle_grid"}, "target");

            Target config{};

            // Required keys
            config.target_type = ToTargetType(Require<std::string>(table, "type"));
            config.size = RequireArray<int, 2>(table, "pattern_size");

            // Optional keys
            OverrideIfPresent(table, "unit_dimension", config.unit_dimension);
            if (auto const circle_grid{OptionalTable(table, "circle_grid")}) {
                RejectUnexpectedKeys(*circle_grid, {"asymmetric"}, "target.circle_grid");
                OverrideIfPresent(*circle_grid, "asymmetric", config.asymmetric);
            }

            return config;
        }

        TargetType target_type;
        std::array<int, 2> size;
        double unit_dimension{1.0};
        bool asymmetric{false};
    };

    static Config Parse(toml::table const& table) {
        RejectUnexpectedKeys(table, {"application", "camera", "imu", "target"}, "");

        return Config{Application::Parse(OptionalTable(table, "application").value_or(toml::table{})),
                      Camera::Parse(RequireTable(table, "camera")),
                      Imu::Parse(OptionalTable(table, "imu").value_or(toml::table{})),
                      Target::Parse(RequireTable(table, "target"))};
    }

    Application application;
    Camera camera;
    std::optional<Imu> imu;
    Target target;
};

}  // namespace reprojection::config

using namespace reprojection;
using namespace std::string_view_literals;

TEST(ConfigParsingHelpers, TestXxxx) {
    static constexpr std::string_view full_table{R"(
        [application]
        show_extraction = true
        threads = 10

        [camera]
        sensor_name = "/cam0/image_raw"
        camera_model = "double_sphere"

        [imu]
        sensor_name = "/imu0"

        [target]
        type = "checkerboard"
        pattern_size = [3,4]

        [target.circle_grid]
        asymmetric = true
    )"};
    toml::table const full_config{toml::parse(full_table)};

    auto const result = config::Config::Parse(full_config);

    EXPECT_EQ(result.application.show_extraction, true);
    EXPECT_EQ(result.application.threads, 10);
    EXPECT_EQ(result.camera.sensor_name, "/cam0/image_raw");
    EXPECT_EQ(result.camera.camera_model, CameraModel::DoubleSphere);
    ASSERT_TRUE(result.imu.has_value());
    EXPECT_EQ(result.imu->sensor_name, "/imu0");
    EXPECT_EQ(result.target.target_type, TargetType::Checkerboard);
    EXPECT_EQ(result.target.size[0], 3);
    EXPECT_EQ(result.target.size[1], 4);
    EXPECT_EQ(result.target.unit_dimension, 1.0);
    EXPECT_EQ(result.target.asymmetric, true);
}