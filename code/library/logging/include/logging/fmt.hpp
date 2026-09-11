#pragma once

#include <ceres/solver.h>
#include <spdlog/fmt/bundled/ranges.h>
#include <spdlog/fmt/fmt.h>

#include <Eigen/Dense>

#include "optimization/bundle_adjustment.hpp"
#include "transforms/rig_state.hpp"
#include "types/database_types.hpp"
#include "types/transform_types.hpp"

// TODO(Jack): For some reason here we are forced here to use the spdlog bundled fmt and not the standard std::format.
// Why that is or if that is a problem is not entirely clear to me, and the general mechanics arround the formatting in
// general scare me.
// TODO(Jack): I am honestly not 100% sure the best practices for such an adapter struct. All I know is that this needs
// to be compiled into the shared object for it to be recognized and used by fmt. Anyone who is more familiar can take
// a look if his becomes a problem.

template <>
struct fmt::formatter<reprojection::StepLogInfo> {
    constexpr auto parse(format_parse_context const& ctx) { return std::cbegin(ctx); }

    template <typename FormatContext>
    auto format(reprojection::StepLogInfo const& info, FormatContext& ctx) const {
        if (info.asset_id) {
            return fmt::format_to(ctx.out(), "'step_type': '{}', 'step_id': {}, 'asset_id': {}", ToString(info.type),
                                  info.step_id.value, info.asset_id->value);
        }

        return fmt::format_to(ctx.out(), "'step_type': '{}', 'step_id': {}", ToString(info.type), info.step_id.value);
    }
};

template <>
struct fmt::formatter<reprojection::Extrinsic> {
    constexpr auto parse(format_parse_context const& ctx) { return std::cbegin(ctx); }

    auto format(reprojection::Extrinsic const& value, format_context& ctx) const {
        auto const& tf{value.se3_a_b};

        return format_to(ctx.out(), R"({{'frame_a': {}, 'frame_b': {}, 'se3_a_b': [{:.3f}]}})", value.frame_a.value,
                         value.frame_b.value, join(tf, ", "));
    }
};

template <>
struct fmt::formatter<reprojection::transforms::RigState> {
    constexpr auto parse(format_parse_context const& ctx) { return std::cbegin(ctx); }

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
        }  // LCOV_EXCL_LINE

        return format_to(out, "]}}");
    }
};

template <>
struct fmt::formatter<reprojection::optimization::BundleAdjustment::CameraState> {
    constexpr auto parse(format_parse_context const& ctx) { return std::cbegin(ctx); }

    auto format(reprojection::optimization::BundleAdjustment::CameraState const& state, format_context& ctx) const {
        return format_to(ctx.out(), "{{'intrinsic': [{:.3f}], 'extrinsic': [{:.3f}]}}",
                         join(state.intrinsic.value, ", "), join(state.extrinsic, ", "));
    }
};

// TODO(Jack): Unit test!
template <>
struct fmt::formatter<reprojection::optimization::BundleAdjustment::Problem> {
    constexpr auto parse(format_parse_context const& ctx) { return std::cbegin(ctx); }

    auto format(reprojection::optimization::BundleAdjustment::Problem const& problem, format_context& ctx) const {
        auto out{format_to(
            ctx.out(), "{{'rig_frame_asset_id': {}, 'num_rig_poses': {}, 'num_observations': {}, 'cameras': [",
            problem.rig_frame_asset_id.value, std::size(problem.rig_poses), std::size(problem.observations))};

        bool first{true};
        for (auto const& [camera_id, camera] : problem.cameras) {
            if (not first) {
                out = format_to(out, ", ");  // LCOV_EXCL_LINE
            }

            // WARN(Jack): For some reason we need to fully qualify fmt::format_to() here and only here, otherwise the
            // camera.state gives us compilation errors!
            out = fmt::format_to(out,
                                 "{{'asset_id': {}, 'camera_model': '{}', 'state': {}, "
                                 "'optimize_intrinsic': {}, 'optimize_extrinsic': {}}}",
                                 camera_id.value, ToString(camera.camera_info.camera_model), camera.state,
                                 camera.options.optimize_intrinsic, camera.options.optimize_extrinsic);
            first = false;
        }

        return format_to(out, "]}}");
    }
};

template <>
struct fmt::formatter<reprojection::optimization::BundleAdjustment::Result> {
    constexpr auto parse(format_parse_context const& ctx) { return std::cbegin(ctx); }

    auto format(reprojection::optimization::BundleAdjustment::Result const& result, format_context& ctx) const {
        auto out{format_to(ctx.out(), "{{'rig_frame_asset_id': {}, 'num_poses': {}, 'camera_states': [",
                           result.rig_frame_asset_id.value, std::size(result.rig_poses))};

        bool first{true};
        for (auto const& [camera_id, camera_state] : result.camera_states) {
            if (not first) {
                out = format_to(out, ", ");
            }

            out = format_to(out, "{{'asset_id': {}, 'state': {}}}", camera_id.value, camera_state);
            first = false;
        }

        return format_to(out, "]}}");
    }
};

template <>
struct fmt::formatter<ceres::Solver::Summary> {
    constexpr auto parse(format_parse_context const& ctx) { return std::cbegin(ctx); }

    auto format(ceres::Solver::Summary const& summary, format_context& ctx) const {
        return format_to(ctx.out(),
                         "{{'initial_cost': {:.2f}, 'final_cost': {:.2f}, 'num_successful_steps': {}, "
                         "'num_unsuccessful_steps': {}, 'num_threads_given': {}, 'num_threads_used': {}, 'msg': {}}}",
                         summary.initial_cost, summary.final_cost, summary.num_successful_steps,
                         summary.num_unsuccessful_steps, summary.num_threads_given, summary.num_threads_used,
                         summary.message);
    }
};