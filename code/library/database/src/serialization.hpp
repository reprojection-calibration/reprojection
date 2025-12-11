#pragma once

#include <optional>

// TODO(Jack): How can we point cppcheck to the generated protobuf cpp files?
// cppcheck-suppress missingInclude
#include "extracted_target.pb.h"
#include "types/calibration_types.hpp"

namespace reprojection::database {

// TODO(Jack): Should this also be optional? Or is the serialization certain to work given a ExtractedTarget?
protobuf_serialization::ExtractedTargetProto Serialize(ExtractedTarget const& target);

std::optional<ExtractedTarget> Deserialize(protobuf_serialization::ExtractedTargetProto const& extracted_target_proto);

inline bool ValidateDimensions(int const rows, int const cols, size_t const data_size) {
    return (static_cast<size_t>(rows) * static_cast<size_t>(cols)) == data_size;
}

}  // namespace reprojection::database