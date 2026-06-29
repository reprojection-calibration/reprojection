#pragma once

#include <format>

#include <toml++/toml.hpp>

#include "types/enums.hpp"

namespace reprojection::config {

std::optional<toml::table> OptionalTable(toml::table const& table, std::string_view key);

toml::table RequireTable(toml::table const& table, std::string_view key);

void RejectUnexpectedKeys(toml::table const& table, std::vector<std::string_view> const& allowed_keys,
                          std::string_view table_name);

template <typename T>
std::optional<T> Optional(toml::table const& table, std::string_view key) {
    toml::node const* const node{table.get(key)};
    if (node == nullptr) {
        return std::nullopt;
    }

    auto const value{node->template value<T>()};
    if (not value) {
        // TODO(Jack): Print out actual expected type and the given value/type.
        throw std::runtime_error(std::format("Invalid type for key '{}'", key));
    }

    return *value;
}

template <typename T>
T Require(toml::table const& table, std::string_view key) {
    auto const value{Optional<T>(table, key)};
    if (not value) {
        throw std::runtime_error( std::format("Missing or invalid required key '{}'", key));
    }

    return *value;
}

template <typename T, size_t N>
std::array<T, N> RequireArray(toml::table const& table, std::string_view key) {
    toml::node const* const node{table.get(key)};
    if (node == nullptr) {
        throw std::runtime_error( std::format("Missing required array '{}'", key));
    }

    toml::array const* const array{node->as_array()};
    if (array == nullptr) {
        throw std::runtime_error( std::format("Invalid type for key '{}' - Expected array", key));
    } else if (array->size() != N) {
        throw std::runtime_error(
             std::format("Invalid array size for key '{}'. Expected {}, got {}", key, N, array->size()));
    }

    std::array<T, N> result{};
    for (size_t i{0}; i < N; ++i) {
        auto const value{(*array)[i].template value<T>()};
        if (not value) {
            throw std::runtime_error( std::format("Invalid type for key '{}[{}]'", key, i));
        }

        result[i] = *value;
    }

    return result;
}

template <typename T>
void OverrideIfPresent(toml::table const& table, std::string_view key, T& value) {
    if (auto const parsed{Optional<T>(table, key)}) {
        value = *parsed;
    }
}

}  // namespace reprojection::config