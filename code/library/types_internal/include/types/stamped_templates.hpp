#pragma once

#include <concepts>
#include <map>

namespace reprojection {

template <typename T>
using StampedData = std::pair<std::uint64_t, T>;

template <typename T>
concept IsStamped = std::same_as<typename T::first_type, std::uint64_t>;

template <typename T>
    requires IsStamped<T>
using StampedMap = std::map<typename T::first_type, typename T::second_type>;

}  // namespace reprojection