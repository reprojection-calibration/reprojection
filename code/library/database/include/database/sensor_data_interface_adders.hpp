#pragma once

#include <memory>

#include "database/calibration_database.hpp"
#include "database/database_data_types.hpp"
#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

// TODO(Jack): Add note in docs that we are hardcoding one target only by not adding a target_id identifier
// TODO(Jack): Add not in docs that we cannot cover all error conditions in unit test so we suppress the errors

namespace reprojection::database {

// RENAME - remove the data suffix
void AddExtractedTargetData(CameraMeasurement const& data, std::string_view sensor_name,
                            std::shared_ptr<CalibrationDatabase> const database);

void AddCameraPoseData(Frames const& data, std::string_view sensor_name, PoseType const type,
                       std::shared_ptr<CalibrationDatabase> const database);

void AddPoseData(Frames const& data, std::string_view sensor_name, PoseType const type, std::string_view sql,
                 std::shared_ptr<CalibrationDatabase> const database);

void AddReprojectionError(std::map<uint64_t, ArrayX2d> const& data, std::string_view sensor_name, PoseType const type,
                          std::shared_ptr<CalibrationDatabase> const database);

[[nodiscard]] bool AddImuData(ImuMeasurement const& data, std::string_view sensor_name,
                              std::shared_ptr<CalibrationDatabase> const database);

}  // namespace reprojection::database