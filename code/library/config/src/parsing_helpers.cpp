#include "parsing_helpers.hpp"

#include <algorithm>
#include <format>

namespace reprojection::config {

std::optional<toml::table> OptionalTable(toml::table const& table, std::string_view key) {
    toml::node const* const node{table.get(key)};
    if (node == nullptr) {
        return std::nullopt;
    }

    toml::table const* const child_table{node->as_table()};
    if (child_table == nullptr) {
        throw std::runtime_error(std::format("'{}' exists but is not a table", key));
    }

    return *child_table;
}

toml::table RequireTable(toml::table const& table, std::string_view key) {
    auto const child_table{OptionalTable(table, key)};
    if (not child_table) {
        throw std::runtime_error(std::format("Missing required table '{}'", key));
    }

    return *child_table;
}

void RejectUnexpectedKeys(toml::table const& table, std::vector<std::string_view> const& allowed_keys,
                          std::vector<std::string_view> const& indexed_keys, std::string_view table_name) {
    for (auto const& [key, _] : table) {
        bool const allowed{
            std::ranges::contains(allowed_keys, key.str()) or
            std::ranges::any_of(indexed_keys, [&](std::string_view base) { return IsIndexedKey(key.str(), base); })};

        if (not allowed) {
            throw std::runtime_error(std::format("Unexpected key '{}.{}'", table_name, key.str()));
        }
    }
}

// Convenience override because for most cases (i.e. anything but the top level table 26.08.2026) the set of indexable
// keys is empty.
void RejectUnexpectedKeys(toml::table const& table, std::vector<std::string_view> const& allowed_keys,
                          std::string_view table_name) {
    return RejectUnexpectedKeys(table, allowed_keys, {}, table_name);
}

// TODO(Jack): Should we use an expected or variant or optional to return the index value from here?
bool IsIndexedKey(std::string_view key, std::string_view base) {
    if (key == base) {
        // Accepts key="cam" and base="cam" - equivalent to index=0 I guess?
        return true;
    } else if (not key.starts_with(base)) {
        return false;
    }

    std::string_view const suffix{key.substr(std::size(base))};
    bool const all_digits{
        std::ranges::all_of(suffix, [](char const c) { return std::isdigit(static_cast<unsigned char>(c)); })};

    return all_digits and not std::empty(suffix);
}

}  // namespace reprojection::config