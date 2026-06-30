#include "config/config_loading.hpp"

#include <format>
#include <variant>

namespace reprojection::config {

// NOTE(Jack): This function basically exists to get more information from the thrown error and provide that directly to
// the user by throwing another error. Why tomlplusplus does not just provide that directly in the message, I do not
// know...
toml::table LoadConfigFile(fs::path const& cfg_path) {
    try {
        return toml::parse_file(cfg_path.string());
    } catch (toml::parse_error const& err) {
        std::string const formatted_error_msg{std::format("{{'file': '{}', 'line': {}, 'error': '{}'}}",
                                                          *err.source().path,       //
                                                          err.source().begin.line,  //
                                                          err.description())};

        throw std::runtime_error(formatted_error_msg);
    }
}

}  // namespace reprojection::config