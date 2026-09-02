#pragma once

#include "types/calibration_types.hpp"

// TODO(Jack): Is there a better name for this file?

namespace reprojection::calibration {

Frames AlignRotations(Frames data);

// This is a very rudimentary sync algorithm! It is not guaranteed to find the minimum total offset sum. If your data is
// way out of sync this might not good for you, but if the data is well synced and sync delta is not too large then it
// will work like you expect.
// TODO(Jack): We do not really need hardcode this for Frames, it can work with any of our stamped types, but for now
// this is all we need.
std::pair<Frames, Frames> SynchronizeFrames(Frames frames_a, Frames frames_b, uint64_t sync_delta_ns);

}  // namespace reprojection::calibration
