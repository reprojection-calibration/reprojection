#pragma once

#include <map>

#include "types/eigen_types.hpp"
#include "types/sensor_types.hpp"

// TODO COMBINE BOTH DATA GENERATORS INTO ONE HEADER FILE?

namespace reprojection::testing_mocks {

// TODO(Jack): Make sure this is not already defined somewhere else, I think it is!

// TODO WHERE TO DEFINE THIS?
using ImuData = std::map<uint64_t, ImuMeasurement>;

// TODO(Jack): I think this generates data in the body frame, i.e. as if it really came from the IMU, and not in some
//  global coordinate frame, but that is something we need to confirm!
ImuData GenerateImuData(int const num_measurements);

}  // namespace reprojection::testing_mocks