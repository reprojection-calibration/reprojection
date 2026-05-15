#pragma once

#include <stdexcept>
#include <string_view>

#include <toml++/toml.hpp>

namespace reprojection::config {

template <typename T>
std::optional<T> ExtractValue(std::string_view key, toml::table& cfg) {
    T value;
    if (auto const node{cfg.get(key)}) {
        value = node->as<T>()->get();
        cfg.erase(key);

        return value;
    } else {
        return std::nullopt;
    }
}  // LCOV_EXCL_LINE

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
        return std::nullopt;
    }

    std::array<T, N> result;
    for (size_t i{0}; i < N; ++i) {
        auto const value{array[i].value<T>()};
        if (not value) {
            return std::nullopt;
        }

        result[i] = *value;
    }

    cfg.erase(key);

    return result;
}

// TODO(Jack): Instead of throwing should we refactor to return a variant with an error message? I think in the config
//  code we do not have a consistent error handling strategy. Sometimes we throw, sometimes we use optional, and
//  sometimes we use variant.
inline void ThrowIfUnexpectedKeys(toml::table const& cfg, std::string_view section) {
    if (cfg.empty()) {
        return;
    }

    std::ostringstream oss;
    oss << "Unexpected parameters found in the " << section << " configuration, are you sure they are correct?\n";
    for (const auto& [key, _] : cfg) {
        oss << "  - " << key.str() << "\n";
    }

    throw std::runtime_error(oss.str());
}

}  // namespace reprojection::config