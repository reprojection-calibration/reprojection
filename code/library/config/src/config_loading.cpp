#include "config/config_loading.hpp"

#include <variant>

namespace reprojection::config {

std::variant<toml::table, TomlErrorMsg> LoadConfigFile(std::string const& file) {
    try {
        return toml::parse_file(file);
    } catch (toml::parse_error const& err) {
        return TomlErrorMsg{TomlError::FailedLoad, "Error parsing file '" + *err.source().path + "' - " +
                                                       std::string(err.description()) + " on line (" +
                                                       std::to_string(err.source().begin.line) + ")"};
    }
}

}  // namespace reprojection::config