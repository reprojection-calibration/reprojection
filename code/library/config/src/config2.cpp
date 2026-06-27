#include "config/config2.hpp"

#include <format>
#include <thread>

#include "config/config_loading.hpp"
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

std::optional<Config> Config::Load(std::filesystem::path const& path) {
    auto const load_result{LoadConfigFile(path)};
    if (std::holds_alternative<TomlErrorMsg>(load_result)) {
        TomlErrorMsg const error{std::get<TomlErrorMsg>(load_result)};
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(error.type), error.msg);

        return std::nullopt;
    }

    toml::table config_table{std::get<toml::table>(load_result)};

    auto const app_table{ExtractTable("application", config_table)};
    if (not app_table) {
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(TomlError::MissingKey),
                   "missing 'application' table");

        return std::nullopt;
    }
    auto const app_result{Application::Parse(*app_table)};
    if (std::holds_alternative<TomlErrorMsg>(app_result)) {
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(std::get<TomlErrorMsg>(app_result).type),
                   std::get<TomlErrorMsg>(app_result).msg);

        return std::nullopt;
    }

    if (auto const result{UnexpectedKeys(config_table)}) {
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(TomlError::UnknownKey), *result);

        return std::nullopt;
    }

    return Config{std::get<Application>(app_result), {}, std::nullopt, {}};
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
        std::string const error_msg{fmt::format("{{'sensor_name': '{}'}}", sensor_name ? *sensor_name : "N/A")};

        return TomlErrorMsg{TomlError::MissingKey, error_msg};
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