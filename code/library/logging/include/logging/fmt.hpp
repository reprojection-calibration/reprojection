#pragma once

#include <spdlog/fmt/fmt.h>

#include <Eigen/Dense>

// TODO(Jack): I am honestly not 100% sure the best practices for such an adapter struct. All I know is that this needs
//  to be compiled into the shared object for it to be recognized and used by fmt. Anyone who is more familiar can take
//  a look if his becomes a problem.

template <>
struct fmt::formatter<Eigen::ArrayXd> {
    char format_specifier = 'f';

    constexpr auto parse(fmt::format_parse_context& ctx) {
        auto it = ctx.begin(), end = ctx.end();
        if (it != end && (*it != '}')) {
            format_specifier = *it++;  // LCOV_EXCL_LINE
        }
        return it;
    }

    template <typename FormatContext>
    auto format(const Eigen::ArrayXd& arr, FormatContext& ctx) const {
        std::string result;
        for (int i = 0; i < arr.size(); ++i) {
            if (i > 0) result += ", ";
            result += fmt::format("{:.3f}", arr[i]);
        }
        return fmt::format_to(ctx.out(), "[{}]", result);
    }
};