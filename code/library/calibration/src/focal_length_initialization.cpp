#include "calibration/focal_length_initialization.hpp"

#include "parabola_line_initialization.hpp"
#include "utilities.hpp"

namespace reprojection::calibration {

// TODO(Jack): Is it really a focal length or a "gamma"?
std::vector<double> InitializeFocalLengthParabolaLine(ExtractedTarget const& target, Vector2d const& principal_point) {
    auto const [rows, cols]{SortIntoRowsAndCols(target)};

    std::vector<double> focal_lengths;

    auto const collect = [&principal_point, &focal_lengths](std::vector<Bundle> const& bundles) {
        for (auto const& bundle : bundles) {
            auto const f{ParabolaLineInitialization(principal_point, bundle.pixels)};
            if (f.has_value()) {
                focal_lengths.push_back(f.value());
            }
        }
    };

    collect(rows);
    collect(cols);

    return focal_lengths;
}

}  // namespace reprojection::calibration
