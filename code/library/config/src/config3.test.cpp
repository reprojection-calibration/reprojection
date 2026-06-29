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
    if (auto parsed = Optional<T>(table, key)) {
        value = *parsed;
    }
}

struct Config {
    struct Application {
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

    static Config Parse(toml::table const& table) {
        return Config{Application::Parse(OptionalTable(table, "application").value_or(toml::table{})),
                      Camera::Parse(RequireTable(table, "camera"))};
    }

    Application application;
    Camera camera;
};

using ParsedValue = std::variant<int64_t, double, bool, std::string, Config::Application>;

using ParseFn = std::function<ParsedValue(toml::table const&)>;

enum class TomlType {
    Array,
    Boolean,
    FloatingPoint,
    Integer,
    String,
    Table,
};

struct ConfigNode {
    std::string key;
    TomlType type;
    bool required{false};
    std::vector<ConfigNode> children{};
    std::optional<ParseFn> parser{};
};

ConfigNode const app_spec{"application",
                          TomlType::Table,
                          false,
                          {{"show_extraction", TomlType::Boolean}, {"threads", TomlType::Integer}},
                          Config::Application::Parse};

ConfigNode const config{"", TomlType::Table, true, {app_spec}};

}  // namespace reprojection::config

using namespace reprojection;
using namespace std::string_view_literals;

TEST(ConfigParsingHelpers, TestXxxx) {
    static constexpr std::string_view full_table{R"(
        [application]
        show_extraction = false
        threads = 1
        unwanted_key = 1

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

    std::cout << result.application.show_extraction << std::endl;
    std::cout << result.application.threads << std::endl;

    EXPECT_EQ(1, 2);
}