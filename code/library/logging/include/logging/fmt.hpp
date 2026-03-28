#pragma once

#include <Eigen/Dense>

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