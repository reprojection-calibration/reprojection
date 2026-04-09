#pragma once

// Forward declaration so we can hide the real db headers from the consuming applications.
namespace reprojection::database {

struct CalibrationDatabase;

using DbPtr = std::shared_ptr<CalibrationDatabase>;

}  // namespace reprojection::database
