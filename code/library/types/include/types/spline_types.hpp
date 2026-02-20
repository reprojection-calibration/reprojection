#pragma once

#include "types/eigen_types.hpp"
#include "types/stamped_templates.hpp"

namespace reprojection {

struct Position {
    Vector3d position;
};
using PositionMeasurement = StampedData<Position>;
using PositionMeasurements = StampedMap<PositionMeasurement>;

struct Velocity {
    Vector3d velocity;
};
using VelocityMeasurement = StampedData<Velocity>;

struct Acceleration {
    Vector3d acceleration;
};
using AccelerationMeasurement = StampedData<Acceleration>;

}  // namespace reprojection