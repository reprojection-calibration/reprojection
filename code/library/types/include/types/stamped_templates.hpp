#pragma once

#include <map>

namespace reprojection {

// TODO(Jack): Use concept to check that the first type for StampedMap is the proper expected stamp type.
template <typename T>
using StampedData = std::pair<std::uint64_t, T>;
template <typename T>
using StampedMap = std::map<typename T::first_type, typename T::second_type>;

}  // namespace reprojection