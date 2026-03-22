#pragma once

#include <stdexcept>
#include <string_view>

#include <toml++/toml.hpp>

namespace reprojection::config {

// ERROR(Jack): This will fail silently if the key is not present in the config! We need a better strategy! Or are we
// counting on that fact the config has been validated before being passed here? If so that depends a lot on the user
// doing the right thing... Bad idea!
template <typename T>
T ExtractValue(std::string_view key, toml::table& cfg) {
    T value;
    if (auto const node{cfg.get(key)}) {
        value = node->as<T>()->get();
        cfg.erase(key);
    }

    return value;
}  // LCOV_EXCL_LINE

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