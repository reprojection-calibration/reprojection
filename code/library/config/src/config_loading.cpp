#include "config/config_loading.hpp"

#include <variant>

namespace reprojection::config {

// TODO(Jack): Use non throwing version of tomlplusplus!!! See below.
std::variant<toml::table, std::string> LoadConfigFile(std::string const& file) {
    try {
        return toml::parse_file(file);
    } catch (toml::parse_error const& err) {
        // TODO(Jack): We could also compile the version of tomlplusplus that does not use exceptions and then the
        //  interface basically does what we are doing here by default.
        return "Error parsing file '" + *err.source().path + "' - " + std::string(err.description()) + " on line (" +
               std::to_string(err.source().begin.line) + ")";
    }
}

}  // namespace reprojection::config