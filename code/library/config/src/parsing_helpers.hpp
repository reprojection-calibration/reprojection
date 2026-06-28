#pragma once

#include <string_view>

#include <toml++/toml.hpp>

#include "config/enums.hpp"
#include "logging/logging.hpp"
#include "types/config.hpp"

namespace reprojection::config {

// TODO(Jack): Clion warning about unnamed namspaces in header files. Is this ok to do this at all? What is a good name?
namespace xxx {

auto const log{logging::Get("config")};

}

// TODO(Jack): Should we return a variant here instead so we can specify the reason for the lack of parsing? Because if
// the value is simply not present that is fine, but if the type is wrong then that is probably actually an error. We
// need to think about how to more cleanly implement this, the current function does not do it right.
template <typename T>
std::optional<T> ExtractValue(std::string_view key, toml::table& cfg) {
    toml::node const* const node{cfg.get(key)};
    if (node == nullptr) {
        return std::nullopt;
    }

    // TODO(Jack): As a temporary solution to not returning a variant here we can instead throw here if the type is bad.
    toml::value<T> const* const value_node{node->as<T>()};
    if (value_node == nullptr) {
        return std::nullopt;
    }

    T const value{value_node->get()};
    cfg.erase(key);

    return value;
}

inline std::optional<toml::table> ExtractTable(std::string_view key, toml::table& cfg) {
    auto const* const node{cfg.get(key)};
    if (not node || not node->is_table()) {
        return std::nullopt;
    }

    auto const table{*node->as_table()};
    cfg.erase(key);

    return table;
}

template <typename T, size_t N>
std::optional<std::array<T, N>> ExtractArray(std::string_view key, toml::table& cfg) {
    auto const* const node{cfg.get(key)};
    if (not node || not node->is_array()) {
        return std::nullopt;
    }

    auto const& array{*node->as_array()};
    if (array.size() != N) {
        return std::nullopt;  // LCOV_EXCL_LINE
    }

    std::array<T, N> result;
    for (size_t i{0}; i < N; ++i) {
        auto const value{array[i].value<T>()};
        if (not value) {
            return std::nullopt;  // LCOV_EXCL_LINE
        }

        result[i] = *value;
    }

    cfg.erase(key);

    return result;
}

template <typename T>
concept IsConfigStruct = requires(toml::table& cfg) {
    { cfg } -> std::same_as<toml::table&>;
    { T::Parse(cfg) } -> std::same_as<std::variant<T, TomlErrorMsg>>;

    { T::TableType() } -> std::same_as<ConfigTable>;
};

// TODO(Jack): How do we handle the case of the imu table which is optional, therefore we do not want to log an error,
// but it also might have errors that we do want to log? At this time (27.06.2026) this is not handled well at all...
template <typename T>
    requires IsConfigStruct<T>
std::optional<T> ParseSubtable(toml::table& main_table) {
    std::string const table_name{ToString(T::TableType())};

    auto const sub_table{ExtractTable(table_name, main_table)};
    if (not sub_table) {
        std::string const msg{fmt::format("{{'toml_error': '{}', 'message': '{}'}}", ToString(TomlError::MissingKey),
                                          std::format("missing required table '{}'", table_name))};
        // TODO(Jack): Hack to prevent the imu log showing up as an error when it is totally normal not to have an IMU
        // table! We need real solution here.
        T::TableType() == ConfigTable::Imu ? xxx::log->debug(msg) : xxx::log->error(msg);

        return std::nullopt;
    }

    auto const parse_result{T::Parse(*sub_table)};
    if (std::holds_alternative<TomlErrorMsg>(parse_result)) {
        xxx::log->error("{{'toml_error': '{}', 'message': '{}'}}",            // LCOV_EXCL_LINE
                        ToString(std::get<TomlErrorMsg>(parse_result).type),  // LCOV_EXCL_LINE
                        std::get<TomlErrorMsg>(parse_result).msg);            // LCOV_EXCL_LINE

        return std::nullopt;  // LCOV_EXCL_LINE
    }

    return std::get<T>(parse_result);
}

std::string ToString(ConfigTable const config_table);

std::optional<std::string> UnexpectedKeys(toml::table const& cfg);

}  // namespace reprojection::config