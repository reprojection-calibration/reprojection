#pragma once

#include <spdlog/fmt/fmt.h>

#include <Eigen/Dense>

#include "transforms/rig_state.hpp"
#include "types/transform_types.hpp"

// TODO(Jack): For some reason here we are forced here to use the spdlog bundled fmt and not the standard std::format.
// Why that is or if that is a problem is not entirely clear to me, and the general mechanics arround the formatting in
// general scare me.
// TODO(Jack): I am honestly not 100% sure the best practices for such an adapter struct. All I know is that this needs
// to be compiled into the shared object for it to be recognized and used by fmt. Anyone who is more familiar can take
// a look if his becomes a problem.

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct fmt::formatter<Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {
    constexpr auto parse(format_parse_context& ctx) { return std::cbegin(ctx); }

    template <typename FormatContext>
    auto format(Eigen::Array<Scalar, Rows, Cols, Options, MaxRows, MaxCols> const& arr, FormatContext& ctx) const {
        auto out{ctx.out()};

        out = format_to(out, "[");
        for (int i{0}; i < arr.size(); ++i) {
            if (i > 0) {
                out = format_to(out, ", ");
            }

            out = format_to(out, "{:.3f}", arr(i));
        }
        out = format_to(out, "]");

        return out;
    }
};

template <>
struct fmt::formatter<reprojection::Extrinsic> {
    constexpr auto parse(format_parse_context& ctx) { return std::cbegin(ctx); }

    auto format(reprojection::Extrinsic const& value, format_context& ctx) const {
        auto const& tf{value.se3_a_b};

        return format_to(ctx.out(), R"({{'frame_a': {}, 'frame_b': {}, 'se3_a_b': [{}, {}, {}, {}, {}, {}]}})",
                         value.frame_a.value, value.frame_b.value, tf[0], tf[1], tf[2], tf[3], tf[4], tf[5]);
    }
};

template <>
struct fmt::formatter<reprojection::transforms::RigState> {
    constexpr auto parse(format_parse_context& ctx) { return std::cbegin(ctx); }

    auto format(reprojection::transforms::RigState const& value, format_context& ctx) const {
        auto out{format_to(ctx.out(), R"({{'rig_frame_asset_id': {}, 'num_poses': {}, 'extrinsics': [)",
                           value.rig_frame_asset_id.value, std::size(value.poses))};

        bool first{true};
        for (auto const& extrinsic : value.extrinsics.Values()) {
            if (not first) {
                out = format_to(out, ", ");
            }

            out = format_to(out, "{}", extrinsic);
            first = false;
        }

        return format_to(out, "]}}");
    }
};