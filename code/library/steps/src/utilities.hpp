#pragma once

namespace reprojection::steps {

template <typename T, typename Logger>
T ValueOrExit(std::expected<T, std::string> const& result, Logger const& log) {
    if (not result) {
        log->error("{}", result.error());

        std::exit(EXIT_FAILURE);
    }

    return *result;
}

}  // namespace reprojection::steps
