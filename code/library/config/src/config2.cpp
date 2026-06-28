#include "config/config2.hpp"

#include <format>
#include <thread>

#include "../include/config/config_loading.hpp"
#include "logging/logging.hpp"

#include "parsing_helpers.hpp"

namespace reprojection::config {

namespace {

auto const log{logging::Get("config")};

}

// TEST AND MOVE
// TEST AND MOVE
// TEST AND MOVE
// TEST AND MOVE
std::optional<std::string> UnexpectedKeys(toml::table const& cfg) {
    if (cfg.empty()) {
        return std::nullopt;
    }

    std::ostringstream oss;
    for (bool first{true}; auto const& [key, value] : cfg) {
        if (first) {
            oss << "{";
            first = false;
        } else {
            oss << ", ";
        }

        oss << "'" << key.str() << "': ";
        value.visit([&oss](auto const& v) { oss << v; });
    }
    oss << "}";

    return oss.str();
}

template <typename T>
concept IsConfigStruct = requires(toml::table& cfg) {
    { cfg } -> std::same_as<toml::table&>;
    { T::Parse(cfg) } -> std::same_as<std::variant<T, TomlErrorMsg>>;

    { T::TableType() } -> std::same_as<ConfigTable>;
};

// TODO(Jack): How do we handle the case of the imu table which is optional, therefore we do not want to log an error,
// but it also might have errors that we do want to log? At this time (27.06.2026) this is not handled well at all...
template <typename T>
    requires IsConfigStruct<T>
std::optional<T> ParseXxx(toml::table& main_table) {
    std::string const table_name{ToString(T::TableType())};

    auto const sub_table{ExtractTable(table_name, main_table)};
    if (not sub_table) {
        std::string const msg{fmt::format("{{'toml_error': '{}', 'message': '{}'}}", ToString(TomlError::MissingKey),
                                          std::format("missing required table '{}'", table_name))};
        // TODO(Jack): Hack to prevent the imu log showing up as an error when it is totally normal not to have an IMU
        // table! We need real solution here.
        T::TableType() == ConfigTable::Imu ? log->debug(msg) : log->error(msg);

        return std::nullopt;
    }

    auto const parse_result{T::Parse(*sub_table)};
    if (std::holds_alternative<TomlErrorMsg>(parse_result)) {
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(std::get<TomlErrorMsg>(parse_result).type),
                   std::get<TomlErrorMsg>(parse_result).msg);

        return std::nullopt;
    }

    return std::get<T>(parse_result);
}

std::optional<Config> Config::Parse(toml::table cfg) {
    auto const app{ParseXxx<Application>(cfg)};
    if (not app) {
        return std::nullopt;
    }

    auto const camera{ParseXxx<Camera>(cfg)};
    if (not camera) {
        return std::nullopt;
    }

    // TODO(Jack): How do we handle the case where there might really be an error with the imu table or it might just
    // not be there because there is no imu data. Does our current code handle this well?
    // Imu is not required which is why we do not return early if nullopt is returned here.
    auto const imu{ParseXxx<Imu>(cfg)};

    auto const target{ParseXxx<Target>(cfg)};
    if (not target) {
        return std::nullopt;
    }

    if (auto const result{UnexpectedKeys(cfg)}) {
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(TomlError::UnknownKey), *result);

        return std::nullopt;
    }

    return Config{*app, *camera, imu, *target};
}

Config::Config(Application const& _app, Camera const& _camera, std::optional<Imu> const& _imu, Target const& _target)
    : app{_app}, camera{_camera}, imu{_imu}, target{_target} {}

std::variant<Config::Application, TomlErrorMsg> Config::Application::Parse(toml::table cfg) {
    auto const show_extraction{ExtractValue<bool>("show_extraction", cfg)};
    auto const threads{ExtractValue<int64_t>("threads", cfg)};

    if (auto const result{UnexpectedKeys(cfg)}) {
        return TomlErrorMsg{TomlError::UnknownKey, *result};
    }

    int const hw_threads{static_cast<int>(std::thread::hardware_concurrency())};

    return Application{show_extraction ? *show_extraction : false,
                       threads ? static_cast<int>(*threads) : std::max(1, hw_threads - 1)};
}

std::variant<Config::Camera, TomlErrorMsg> Config::Camera::Parse(toml::table cfg) {
    auto const sensor_name{ExtractValue<std::string>("sensor_name", cfg)};
    auto const camera_model{ExtractValue<std::string>("camera_model", cfg)};

    if (not sensor_name or not camera_model) {
        std::string const error_msg{fmt::format("{{'sensor_name': '{}', 'camera_model': '{}'}}",
                                                sensor_name ? *sensor_name : "N/A",
                                                camera_model ? *camera_model : "N/A")};

        return TomlErrorMsg{TomlError::MissingKey, error_msg};
    }

    if (auto const result{UnexpectedKeys(cfg)}) {
        return TomlErrorMsg{TomlError::UnknownKey, *result};
    }

    return Camera{*sensor_name, ToCameraModel(*camera_model)};
}

std::variant<Config::Imu, TomlErrorMsg> Config::Imu::Parse(toml::table cfg) {
    auto const sensor_name{ExtractValue<std::string>("sensor_name", cfg)};

    if (not sensor_name) {
        return TomlErrorMsg{TomlError::MissingKey, "{'sensor_name': 'N/A'}"};
    }

    if (auto const result{UnexpectedKeys(cfg)}) {
        return TomlErrorMsg{TomlError::UnknownKey, *result};
    }

    return Imu{*sensor_name};
}

std::variant<Config::Target, TomlErrorMsg> Config::Target::Parse(toml::table cfg) {
    auto const type{ExtractValue<std::string>("type", cfg)};
    auto const pattern_size{ExtractArray<int, 2>("pattern_size", cfg)};

    if (not type or not pattern_size) {
        std::string const error_msg{
            fmt::format("{{'type': '{}', 'pattern_size': '{}'}}", type ? *type : "N/A",
                        pattern_size ? fmt::format("[{}, {}]", (*pattern_size)[0], (*pattern_size)[1]) : "N/A")};
        return TomlErrorMsg{TomlError::MissingKey, error_msg};
    }

    // Optional value for all target types
    auto const unit_dimension{ExtractValue<double>("unit_dimension", cfg)};

    // Optional value only for circle grid targets
    TargetType const target_type{ToTargetType(*type)};
    std::optional<bool> asymmetric;
    if (auto circle_grid_cfg{ExtractTable("circle_grid", cfg)}) {
        asymmetric = ExtractValue<bool>("asymmetric", *circle_grid_cfg);

        if (auto const result{UnexpectedKeys(*circle_grid_cfg)}) {
            return TomlErrorMsg{TomlError::UnknownKey, *result};
        }
    }

    if (auto const result{UnexpectedKeys(cfg)}) {
        return TomlErrorMsg{TomlError::UnknownKey, *result};
    }

    return Target{target_type, *pattern_size, unit_dimension ? *unit_dimension : 1.0, asymmetric ? *asymmetric : false};
}

}  // namespace reprojection::config