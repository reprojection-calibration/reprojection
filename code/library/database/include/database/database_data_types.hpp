#pragma once

#include <string>

namespace reprojection::database {

enum class PoseTable { Camera, External };

enum class PoseType { GroundTruth, Initial, Optimized };

// TODO(Jack): Where does this belong?
inline std::string ToString(PoseType const t) {
    switch (t) {
        case PoseType::GroundTruth:
            return "ground_truth";
        case PoseType::Initial:
            return "initial";
        case PoseType::Optimized:
            return "optimized";
    }

    throw std::logic_error("Invalid PoseType");  // LCOV_EXCL_LINE
}

}  // namespace reprojection::database