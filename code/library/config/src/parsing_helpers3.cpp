#include "parsing_helpers3.hpp"

#include <algorithm>

namespace reprojection::config {

std::optional<toml::table> OptionalTable(toml::table const& table, std::string_view key) {
    toml::node const* const node{table.get(key)};
    if (node == nullptr) {
        return std::nullopt;
    }

    toml::table const* const child_table{node->as_table()};
    if (child_table == nullptr) {
        throw std::runtime_error(fmt::format("'{}' exists but is not a table.", key));
    }

    return *child_table;
}

toml::table RequireTable(toml::table const& table, std::string_view key) {
    auto const child_table{OptionalTable(table, key)};
    if (not child_table) {
        throw std::runtime_error(fmt::format("Missing required table '{}'.", key));
    }

    return *child_table;
}

void RejectUnexpectedKeys(toml::table const& table, std::vector<std::string_view> const& allowed_keys,
                          std::string_view table_name) {
    for (auto const& [key, _] : table) {
        bool const allowed{
            std::ranges::any_of(allowed_keys, [&](std::string_view allowed_key) { return key.str() == allowed_key; })};

        if (not allowed) {
            throw std::runtime_error(fmt::format("Unexpected key '{}.{}'.", table_name, key.str()));
        }
    }
}

}  // namespace reprojection::config