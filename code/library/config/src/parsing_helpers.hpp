#pragma once

#include <stdexcept>
#include <string_view>

#include <toml++/toml.hpp>

namespace reprojection::config {

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

// TODO(Jack): Instead of throwing should we refactor to return a variant with an error message? I think in the config
//  code we do not have a consistent error handling strategy. Sometimes we throw, sometimes we use optional, and
//  sometimes we use variant.
inline void ThrowIfUnexpectedKeys(toml::table const& cfg) {
    if (cfg.empty()) {
        return;
    }

    std::ostringstream oss;
    oss << "Unexpected configuration parameters found., are you sure they are correct?\n";
    for (const auto& [key, _] : cfg) {
        oss << "  - " << key.str() << "\n";
    }

    throw std::runtime_error(oss.str());
}

}  // namespace reprojection::config