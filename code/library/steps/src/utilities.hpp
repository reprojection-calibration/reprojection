#pragma once

namespace reprojection::steps {

template <typename T, typename Logger>
T ValueOrExit(std::expected<T, std::string> const& result, Logger const& log) {
    if (not result) {
        log->error("{}", result.error());  // LCOV_EXCL_LINE

        std::exit(EXIT_FAILURE);  // LCOV_EXCL_LINE
    }

    return *result;
}

}  // namespace reprojection::steps
