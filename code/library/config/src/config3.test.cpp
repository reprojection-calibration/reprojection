#include <gtest/gtest.h>

#include <spdlog/fmt/bundled/format.h>  // TODO REMOVE?

#include <thread>

#include <toml++/toml.hpp>

#include "types/enums.hpp"

namespace reprojection::config {

template <typename T>
std::optional<T> Optional(toml::table const& table, std::string_view key) {
    toml::node const* node = table.get(key);

    if (node == nullptr) {
        return std::nullopt;
    }

    auto value = node->template value<T>();

    if (!value) {
        // TODO(Jack): Print out actual type and value.
        throw std::runtime_error(fmt::format("Invalid type for key '{}'.", key));
    }

    return *value;
}

template <typename T>
T Require(toml::table const& table, std::string_view key) {
    auto const value = Optional<T>(table, key);

    if (not value) {
        throw std::runtime_error(fmt::format("Missing or invalid required key '{}'.", key));
    }

    return *value;
}

std::optional<toml::table> OptionalTable(toml::table const& table, std::string_view key) {
    toml::node const* node = table.get(key);

    if (node == nullptr) {
        return std::nullopt;
    }

    toml::table const* child_table = node->as_table();

    if (child_table == nullptr) {
        throw std::runtime_error(fmt::format("'{}' exists but is not a table.", key));
    }

    return *child_table;
}

toml::table RequireTable(toml::table const& table, std::string_view key) {
    auto child_table = OptionalTable(table, key);

    if (!child_table) {
        throw std::runtime_error(fmt::format("Missing required table '{}'.", key));
    }

    return *child_table;
}

template <typename T>
void OverrideIfPresent(toml::table const& table, std::string_view key, T& value) {
    if (auto const parsed{Optional<T>(table, key)}) {
        value = *parsed;
    }
}

// TODO(Jack): Reject unexpected keys!
struct Config {
    struct Application {
        // The table is not required, but we have sensible defaults.
        static Application Parse(toml::table const& table) {
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
            return Camera{Require<std::string>(table, "sensor_name"),
                          ToCameraModel(Require<std::string>(table, "camera_model"))};
        }

        std::string sensor_name;
        CameraModel camera_model;
    };

    struct Imu {
        // The table is not required, but we have no sensible defaults.
        static std::optional<Imu> Parse(toml::table const& table) {
            auto sensor_name{Optional<std::string>(table, "sensor_name")};
            if (not sensor_name) {
                return std::nullopt;
            }

            return Imu{*sensor_name};
        }

        std::string sensor_name;
    };

    static Config Parse(toml::table const& table) {
        return Config{Application::Parse(OptionalTable(table, "application").value_or(toml::table{})),
                      Camera::Parse(RequireTable(table, "camera")),
                      Imu::Parse(OptionalTable(table, "imu").value_or(toml::table{}))};
    }

    Application application;
    Camera camera;
    std::optional<Imu> imu;
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
    )"};
    toml::table const full_config{toml::parse(full_table)};

    auto const result = config::Config::Parse(full_config);

    EXPECT_EQ(result.application.show_extraction, true);
    EXPECT_EQ(result.application.threads, 10);
    EXPECT_EQ(result.camera.sensor_name, "/cam0/image_raw");
    EXPECT_EQ(result.camera.camera_model, CameraModel::DoubleSphere);
    ASSERT_TRUE(result.imu.has_value());
    EXPECT_EQ(result.imu->sensor_name, "/imu0");
    // EXPECT_EQ(result.target.target_type, TargetType::Checkerboard);
    // EXPECT_EQ(result.target.size[0], 3);
    // EXPECT_EQ(result.target.size[1], 4);
    // EXPECT_EQ(result.imu->sensor_name, "/imu0");

    EXPECT_EQ(1, 2);
}