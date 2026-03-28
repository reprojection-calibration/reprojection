#pragma once

#include <spdlog/cfg/env.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <memory>

namespace reprojection::logging {

inline std::shared_ptr<spdlog::logger> get(std::string const& name) {
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

// TODO MOVE TO FILE
// TODO MOVE TO FILE
// TODO MOVE TO FILE
// TODO MOVE TO FILE
inline std::string CompactPrettyJson(const std::string& input) {
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
inline std::string ToOneLineJson(const toml::table& tbl) {
    std::ostringstream oss;
    oss << toml::json_formatter{tbl};

    return CompactPrettyJson(oss.str());
}

}  // namespace reprojection::logging

// TODO CLEAN UP!!!
template <>
struct fmt::formatter<Eigen::ArrayXd> {
    char format_specifier = 'f';

    constexpr auto parse(fmt::format_parse_context& ctx) {
        auto it = ctx.begin(), end = ctx.end();
        if (it != end && (*it != '}')) {
            format_specifier = *it++;
        }
        return it;
    }

    template <typename FormatContext>
    auto format(const Eigen::ArrayXd& arr, FormatContext& ctx) {
        std::string result;
        for (int i = 0; i < arr.size(); ++i) {
            if (i > 0) result += ", ";
            result += fmt::format("{:.3f}", arr[i]);
        }
        return fmt::format_to(ctx.out(), "[{}]", result);
    }
};