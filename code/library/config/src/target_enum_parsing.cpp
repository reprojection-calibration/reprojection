#include "target_enum_parsing.hpp"

#include <stdexcept>

namespace reprojection::config {

TargetType StringToTargetTypeEnum(std::string const& enum_string) {
    if (enum_string == "checkerboard") {
        return TargetType::Checkerboard;
    } else if (enum_string == "circle_grid") {
        return TargetType::CircleGrid;
    } else if (enum_string == "april_grid3") {
        return TargetType::AprilGrid3;
    } else {
        throw std::runtime_error("The requested target type string - " + enum_string +
                                 " - is not valid (see the TargetType enum).");
    }
}

}  // namespace reprojection::config