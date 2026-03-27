#include "application/io.hpp"

#include "config/config_loading.hpp"
#include "config/config_validation.hpp"

#include "logging.hpp"

namespace reprojection::application {

namespace {

auto const log{logging::get("application")};

}

std::optional<PathConfig> ParseCommandLineInput(int const argc, char const* const argv[]) {
    auto const config_path{GetCommandOption(argv, argv + argc, "--config")};
    if (not config_path) {
        log->error("Missing --config flag");
        return std::nullopt;
    }

    auto const data_path{GetCommandOption(argv, argv + argc, "--data")};
    if (not data_path) {
        log->error("Missing --data flag");
        return std::nullopt;
    }

    PathConfig path_config{*config_path, *data_path, ""};

    // If a workspace directory is provided use it, otherwise just use the directory where the data is.
    auto const workspace_dir{GetCommandOption(argv, argv + argc, "--workspace")};
    path_config.workspace_dir = workspace_dir ? fs::path{*workspace_dir} : path_config.data_path.parent_path();

    log->info("{{'config_path': '{}', 'data_path': '{}', 'workspace_dir': '{}'}}", path_config.config_path.string(),
              path_config.data_path.string(), path_config.workspace_dir.string());

    return path_config;
}

std::optional<std::string> GetCommandOption(char const* const* const begin, char const* const* const end,
                                            std::string const& option) {
    char const* const* itr{std::find(begin, end, option)};
    if (itr != end and ++itr != end) {
        return std::string(*itr);
    }

    return std::nullopt;
}

// TODO MOVE TO FILE
// TODO MOVE TO FILE
// TODO MOVE TO FILE
// TODO MOVE TO FILE
std::string CompactPrettyJson(const std::string& input) {
    std::string out;
    out.reserve(input.size());

    bool in_string = false;
    bool escape = false;

    for (size_t i = 0; i < input.size(); ++i) {
        char c = input[i];

        if (escape) {
            out += c;
            escape = false;
            continue;
        }

        if (c == '\\') {
            out += c;
            escape = true;
            continue;
        }

        if (c == '"') {
            in_string = !in_string;
            out += "'";
            continue;
        }

        if (!in_string) {
            if (std::isspace(static_cast<unsigned char>(c))) continue;

            // Add normalized spacing so it is more readable
            if (c == ':') {
                out += ": ";
                continue;
            }
            if (c == ',') {
                out += ", ";
                continue;
            }
        }

        out += c;
    }

    return out;
}

// TODO MOVE TO FILE
// TODO MOVE TO FILE// TODO MOVE TO FILE
// TODO MOVE TO FILE
std::string ToOneLineJson(const toml::table& tbl) {
    std::ostringstream oss;
    oss << toml::json_formatter{tbl};

    return CompactPrettyJson(oss.str());
}

std::optional<toml::table> LoadAndValidateConfig(fs::path const& config_path) {
    auto const loaded_config{config::LoadConfigFile(config_path)};
    if (std::holds_alternative<TomlErrorMsg>(loaded_config)) {
        auto const error_msg{std::get<TomlErrorMsg>(loaded_config)};
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(error_msg.error), error_msg.msg);
        return std::nullopt;
    }

    if (auto const error_msg{config::ValidateCalibrationConfig(std::get<toml::table>(loaded_config))}) {
        log->error("{{'toml_error': '{}', 'message': '{}'}}", ToString(error_msg->error), error_msg->msg);
        return std::nullopt;
    }

    auto const config{std::get<toml::table>(loaded_config)};
    log->info("{{'config_path': '{}', 'config': {}}}", config_path.string(), ToOneLineJson(config));

    return config;
}

}  // namespace reprojection::application
