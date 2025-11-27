#include "calibration/initialize_focal_length.hpp"

#include <set>

#include "eigen_utilities/grid.hpp"
#include "parabola_line_initialization.hpp"
#include "vanishing_point_initialization.hpp"

namespace reprojection::calibration {

// NOTE(Jack): Principle point only required for ParabolaLineInitialization, is there anyway to prevent this?
std::vector<double> InitializeFocalLength(ExtractedTarget const& target, InitializationMethod const method,
                                          Vector2d const& principal_point) {
    auto const [rows, cols]{calibration::SortIntoRowsAndCols(target)};

    std::vector<double> focal_lengths;
    if (method == InitializationMethod::ParabolaLine) {
        // TODO(Jack): Do we really need to iterate over the rows and columns? Or would just the rows or just the
        // columns already be enough?
        for (auto const& row : rows) {
            auto f{calibration::ParabolaLineInitialization(principal_point, row.pixels)};
            if (f.has_value()) {
                focal_lengths.push_back(f.value());
            }
        }
        for (auto const& col : cols) {
            auto f{ParabolaLineInitialization(principal_point, col.pixels)};
            if (f.has_value()) {
                focal_lengths.push_back(f.value());
            }
        }
    } else if (method == InitializationMethod::VanishingPoint) {
        for (auto const& row : rows) {
            for (auto const& col : cols) {
                auto const f{VanishingPointInitialization(row.pixels, col.pixels)};
                if (f.has_value()) {
                    focal_lengths.push_back(f.value());
                }
            }
        }
    }

    return focal_lengths;
}

std::tuple<std::vector<Bundle>, std::vector<Bundle>> SortIntoRowsAndCols(ExtractedTarget const& target) {
    // Get rows
    // See https://stackoverflow.com/questions/14740867/find-unique-numbers-in-array
    std::set<int> const row_ids{target.indices.col(0).cbegin(), target.indices.col(0).cend()};
    std::vector<Bundle> rows;
    for (auto const id : row_ids) {
        ArrayXi const row_mask{
            (target.indices.col(0).unaryExpr([id](int const x) { return x == id; }) == true).cast<int>()};
        ArrayXi const indices{eigen_utilities::MaskIndices(row_mask)};

        rows.push_back(target.bundle(indices));
    }

    // TODO(Jack): Can we avoid the copy and paste?
    // Get cols
    std::set<int> const col_ids{target.indices.col(1).cbegin(), target.indices.col(1).cend()};
    std::vector<Bundle> cols;
    for (auto const id : col_ids) {
        ArrayXi const col_mask{
            (target.indices.col(1).unaryExpr([id](int const x) { return x == id; }) == true).cast<int>()};
        ArrayXi const indices{eigen_utilities::MaskIndices(col_mask)};

        cols.push_back(target.bundle(indices));
    }

    return {rows, cols};
}

}  // namespace reprojection::calibration