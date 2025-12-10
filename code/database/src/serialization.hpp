#pragma once

#include <optional>

#include "extracted_target.pb.h"
#include "types/calibration_types.hpp"

namespace reprojection::database {

// TODO(Jack): Should this also be optional? Or is the serialization certain to work given a ExtractedTarget?
protobuf_serialization::ExtractedTargetProto Serialize(ExtractedTarget const& target);

std::optional<ExtractedTarget> Deserialize(protobuf_serialization::ExtractedTargetProto const& proto);

}  // namespace reprojection::database