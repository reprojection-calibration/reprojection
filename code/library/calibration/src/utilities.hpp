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
    std::vector<typename Map::value_type> sampled;

    std::sample(std::cbegin(map), std::cend(map), std::back_inserter(sampled), std::min(num, std::size(map)),
                std::mt19937{std::random_device{}()});

    Map result;
    for (auto const& sample : sampled) {
        result.insert(sample);
    }

    return result;
}

}  // namespace reprojection::calibration