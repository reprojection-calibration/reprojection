#include "logging/logging.hpp"

#include <spdlog/cfg/env.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <memory>

#include "formatting.hpp"

namespace reprojection::logging {

std::shared_ptr<spdlog::logger> Get(std::string const& name) {
    auto logger{spdlog::get(name)};
    if (not logger) {
        spdlog::cfg::load_env_levels();

        logger = spdlog::stdout_color_mt(name);

        char const* const log_pattern{std::getenv("SPDLOG_PATTERN")};
        std::string const pattern{log_pattern ? log_pattern : "[%H:%M:%S] [%n] [%l] %v"};
        logger->set_pattern(pattern);
    }

    return logger;
}

std::string ToOneLineJson(const toml::table& tbl) {
    std::ostringstream oss;
    oss << toml::json_formatter{tbl};

    return CompactPrettyJson(oss.str());
}

}  // namespace reprojection::logging
