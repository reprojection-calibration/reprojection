#pragma once

#include <spdlog/spdlog.h>

#include <toml++/toml.hpp>

namespace reprojection::logging {

std::shared_ptr<spdlog::logger> Get(std::string const& name);

std::string ToOneLineJson(toml::table const& tbl);

// TODO(Jack): There is a fmt::join() function that does exactly this on ubuntu 24.04! The only problem is that it does
//  not exist on ubuntu 20.04 in the apt package default. I really do not think a source compile, or messing around with
//  versions is worth it, so we just hacked together this helper method.
template <typename Container>
std::string Join(Container const& c, std::string const& sep) {
    std::ostringstream oss;
    auto it{c.begin()};
    if (it != c.end()) {
        oss << *it++;
    }
    while (it != c.end()) {
        oss << sep << *it++;
    }
    return oss.str();
}

}  // namespace reprojection::logging
