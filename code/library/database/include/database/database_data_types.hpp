#pragma once

#include <string>

namespace reprojection::database {

// TODO(Jack): At one point in time we had an "External" poses table for external groundtruth but then this was removed.
//  We should work to remove all traces of the development from the code base.
enum class PoseTable { Camera };

enum class PoseType { Initial, Optimized };

// TODO(Jack): Where does this belong?
inline std::string ToString(PoseType const t) {
    switch (t) {
        case PoseType::Initial:
            return "initial";
        case PoseType::Optimized:
            return "optimized";
    }

    throw std::logic_error("Invalid PoseType");  // LCOV_EXCL_LINE
}

}  // namespace reprojection::database