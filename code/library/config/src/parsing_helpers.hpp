#pragma once

#include <format>

#include <toml++/toml.hpp>

#include "types/enums.hpp"

namespace reprojection::config {

// NOTE(Jack): I chose to throw errors everywhere because using variants and passing error messages around everywhere
// was burdensome and made refactoring difficult. Because all these config errors are important enough that the program
// needs to be killed when thrown and cannot be handled anyway, I think it is ok to use throwing here.

std::optional<toml::table> OptionalTable(toml::table const& table, std::string_view key);

toml::table RequireTable(toml::table const& table, std::string_view key);

std::optional<int> IndexedKeyIndex(std::string_view key, std::string_view base);

// TODO ADD CONCEPT THAT PARSE METHOD IS REQUIRED?
template <typename T>
std::vector<T> RequireIndexedTables(toml::table const& table, std::string_view base) {
    std::vector<T> result;
    for (auto const& [key, value] : table) {
        std::optional const index{IndexedKeyIndex(key.str(), base)};
        if (not index) {
            continue;
        }

        result.push_back(T::Parse(RequireTable(table, key), *index));
    }

    if (std::empty(result)) {
        throw std::runtime_error(std::format("Missing required table(s) '{}'", base));
    }

    return result;
}

void RejectUnexpectedKeys(toml::table const& table, std::vector<std::string_view> const& allowed_keys,
                          std::vector<std::string_view> const& indexed_keys, std::string_view table_name);

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
        throw std::runtime_error(std::format("Missing or invalid required key '{}'", key));
    }

    return *value;
}

template <typename T, size_t N>
std::array<T, N> RequireArray(toml::table const& table, std::string_view key) {
    toml::node const* const node{table.get(key)};
    if (node == nullptr) {
        throw std::runtime_error(std::format("Missing required array '{}'", key));
    }

    toml::array const* const array{node->as_array()};
    if (array == nullptr) {
        throw std::runtime_error(std::format("Invalid type for key '{}' - Expected array", key));
    } else if (array->size() != N) {
        throw std::runtime_error(
            std::format("Invalid array size for key '{}'. Expected {}, got {}", key, N, array->size()));
    }

    std::array<T, N> result{};
    for (size_t i{0}; i < N; ++i) {
        auto const value{(*array)[i].template value<T>()};
        if (not value) {
            throw std::runtime_error(std::format("Invalid type for key '{}[{}]'", key, i));
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