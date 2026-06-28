#include "parsing_helpers.hpp"

namespace reprojection::config {

std::string ToString(ConfigTable const config_table) {
    if (config_table == ConfigTable::Application) {
        return "application";
    } else if (config_table == ConfigTable::Camera) {
        return "camera";
    } else if (config_table == ConfigTable::Imu) {
        return "imu";
    } else if (config_table == ConfigTable::Target) {
        return "target";
    } else {
        throw std::runtime_error(
            "LIBRARY IMPLEMENTATION ERROR - Unrecognized argument passed to ToString(ConfigTable)");
    }
}

// TODO(Jack): Test!
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

}  // namespace reprojection::config