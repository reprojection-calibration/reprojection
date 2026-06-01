#pragma once

#include <map>

#include "spline/se3_spline.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::testing_mocks {

// TODO(Jack): I think this generates data in the body frame, i.e. as if it really came from the IMU, and not in some
//  global coordinate frame, but that is something we need to confirm!
std::pair<ImuMeasurements, spline::Se3Spline> GenerateImuData(int const num_samples, uint64_t const timespan_ns);

}  // namespace reprojection::testing_mocks