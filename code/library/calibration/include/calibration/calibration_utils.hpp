#pragma once

#include "types/calibration_types.hpp"

// TODO(Jack): Is there a better name for this file?

namespace reprojection::calibration {

Frames AlignRotations(Frames data);

std::pair<Frames, Frames> SynchronizeFrames(Frames frames_a, Frames frames_b, uint64_t sync_delta_ns);

}  // namespace reprojection::calibration
