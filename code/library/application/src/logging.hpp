#pragma once

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>

namespace reprojection::logging {

inline std::shared_ptr<spdlog::logger> get(std::string const& name) {
    auto logger{spdlog::get(name)};
    if (not logger) {
        logger = spdlog::stdout_color_mt(name);
        logger->set_pattern("[%H:%M:%S] [%n] [%l] %v");
    }
    return logger;
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