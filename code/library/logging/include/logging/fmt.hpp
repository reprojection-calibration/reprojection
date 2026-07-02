#pragma once

#include <spdlog/fmt/fmt.h>

#include <Eigen/Dense>

// TODO(Jack): I am honestly not 100% sure the best practices for such an adapter struct. All I know is that this needs
//  to be compiled into the shared object for it to be recognized and used by fmt. Anyone who is more familiar can take
//  a look if his becomes a problem.

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct fmt::formatter<Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols> const& arr, FormatContext& ctx) const {
        auto out{ctx.out()};

        out = fmt::format_to(out, "[");
        for (int i{0}; i < arr.size(); ++i) {
            if (i > 0) {
                out = fmt::format_to(out, ", ");
            }

            out = fmt::format_to(out, "{:.3f}", arr(i));
        }
        out = fmt::format_to(out, "]");

        return out;
    }
};