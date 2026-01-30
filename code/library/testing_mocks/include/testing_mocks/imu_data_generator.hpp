#pragma once

#include <map>

#include "types/eigen_types.hpp"

// TODO COMBINE BOTH DATA GENERATORS INTO ONE HEADER FILE?

namespace reprojection::testing_mocks {

// TODO(Jack): Make sure this is not already defined somewhere else, I think it is!
struct ImuMeasurement {
    Vector3d angular_velocity;
    Vector3d linear_acceleration;
};

// TODO WHERE TO DEFINE THIS?
using ImuData = std::map<uint64_t, ImuMeasurement>;

ImuData GenerateImuData(int const num_measurements);

}  // namespace reprojection::testing_mocks