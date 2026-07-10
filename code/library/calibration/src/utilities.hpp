#pragma once

#include <random>
#include <vector>

#include "types/algorithm_types.hpp"

namespace reprojection::calibration {

enum class Dimension { Row = 0, Col = 1 };

std::pair<std::vector<Bundle>, std::vector<Bundle>> SortIntoRowsAndCols(ExtractedTarget const& target);

std::vector<Bundle> ExtractBundlesByDimension(ExtractedTarget const& target, Dimension dim);

template <typename Map>
auto SampleMap(Map const& map, std::size_t const num) {
    auto const size{std::size(map)};
    auto const sample_count{std::min(num, size)};

    Map result;
    if (sample_count == 0) {
        return result;
    } else if (sample_count >= size) {
        return map;
    } else if (sample_count == 1) {
        auto it{std::cbegin(map)};
        std::advance(it, static_cast<std::ptrdiff_t>(size / 2));
        result.insert(*it);

        return result;
    }

    // Sample at evenly spaced intervals
    auto it{std::cbegin(map)};
    std::size_t current_index{0};
    for (std::size_t i{0}; i < sample_count; ++i) {
        auto const target_index{static_cast<std::size_t>(std::round(
            static_cast<double>(i) * static_cast<double>(size - 1) / static_cast<double>(sample_count - 1)))};

        std::advance(it, static_cast<std::ptrdiff_t>(target_index - current_index));
        current_index = target_index;

        result.insert(*it);
    }

    return result;
}

}  // namespace reprojection::calibration