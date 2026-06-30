#include "config/config_parse.hpp"

#include "parsing_helpers.hpp"

namespace reprojection::config {

// TODO(Jack): Should we define an enum and a ToString() method for the top level required tables? I see throughout the
// code "application" or "camera" as strings copy and pasted a little too much for comfort.
// TODO(Jack): Is there anyway for us to specify the config in a graph and then construct the parsing code automatically
// from the graph? The current setup works but everytime we need to add a new parameter we will need to touch the code
// at multiple places which is not so nice. Not a deal breaker but consider it!

Config Config::Parse(toml::table const& table) {
    RejectUnexpectedKeys(table, {"application", "camera", "imu", "target"}, "");

    return Config{Application::Parse(OptionalTable(table, "application").value_or(toml::table{})),
                  Camera::Parse(RequireTable(table, "camera")),
                  Imu::Parse(OptionalTable(table, "imu").value_or(toml::table{})),
                  Target::Parse(RequireTable(table, "target"))};
}

// The table is not required, but we have sensible defaults.
Config::Application Config::Application::Parse(toml::table const& table) {
    RejectUnexpectedKeys(table, {"show_extraction", "threads"}, "application");

    Application config{};
    OverrideIfPresent(table, "show_extraction", config.show_extraction);
    OverrideIfPresent(table, "threads", config.threads);

    return config;
}

Config::Camera Config::Camera::Parse(toml::table const& table) {
    RejectUnexpectedKeys(table, {"sensor_name", "camera_model"}, "camera");

    return Camera{Require<std::string>(table, "sensor_name"),
                  ToCameraModel(Require<std::string>(table, "camera_model"))};
}

// The table is not required (we do not always have IMU data), but we have no sensible defaults.
std::optional<Config::Imu> Config::Imu::Parse(toml::table const& table) {
    RejectUnexpectedKeys(table, {"sensor_name"}, "imu");

    auto const sensor_name{Optional<std::string>(table, "sensor_name")};
    if (not sensor_name) {
        return std::nullopt;
    }

    return Imu{*sensor_name};
}

// TODO(Jack): The target config is essentially the exact same as the TargetInfo type, we need top unify these! I think
// by doing that we can remove the target info step entirely from the calibration process because the entire description
// is contained in the config file itself and can be loaded directly.
Config::Target Config::Target::Parse(toml::table const& table) {
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

}  // namespace reprojection::config