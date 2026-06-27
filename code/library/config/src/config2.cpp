#include "config/config2.hpp"

#include <iostream>  // REMOVE
#include <thread>

#include "config/config_loading.hpp"
#include "config/config_validation.hpp"
#include "logging/logging.hpp"

#include "parsing_helpers.hpp"

namespace reprojection::config {

namespace {

auto const log{logging::Get("config")};

}

std::optional<Config> Config::Load(std::filesystem::path const& path) {
    auto const load_result{LoadConfigFile(path)};
    if (std::holds_alternative<TomlErrorMsg>(load_result)) {
        TomlErrorMsg const error{std::get<TomlErrorMsg>(load_result)};
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(error.type), error.msg);

        return std::nullopt;
    }

    toml::table const toml_table{std::get<toml::table>(load_result)};
    if (auto const error{ValidateCalibrationConfig(toml_table)}) {
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(error->type), error->msg);

        return std::nullopt;
    }

    // Now that we have validated the config we have a guarantee that the required_keys are present and therefore can
    // access them directly.

    auto const app_config{Application::Parse(*toml_table["application"].as_table())};

    return std::nullopt;
}

// TEST AND MOVE
std::optional<std::string> UnexpectedKeys(toml::table const& cfg) {
    if (cfg.empty()) {
        return std::nullopt;
    }

    std::ostringstream oss;
    oss << "Unexpected keys";
    for (const auto& [key, value] : cfg) {
        oss << "  - " << key.str();
    }

    return oss.str();
}

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

}  // namespace reprojection::config