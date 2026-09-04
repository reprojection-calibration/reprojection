#pragma once

#include "types/eigen_types.hpp"
#include "types/stamped_templates.hpp"

namespace reprojection {

struct Position {
    Vector3d value;
};

using PositionSample = StampedData<Position>;
using PositionSamples = StampedMap<PositionSample>;

struct Velocity {
    Vector3d value;
};

using VelocitySample = StampedData<Velocity>;
using VelocitySamples = StampedMap<VelocitySample>;

struct Acceleration {
    Vector3d value;
};

using AccelerationSample = StampedData<Acceleration>;
using AccelerationSamples = StampedMap<AccelerationSample>;

}  // namespace reprojection