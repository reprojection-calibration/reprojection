#pragma once

#include <memory>

#include <opencv2/opencv.hpp>

#include "database/calibration_database.hpp"
#include "database/database_data_types.hpp"
#include "types/calibration_types.hpp"
#include "types/sensor_types.hpp"

// TODO(Jack): Add note in docs that we are hardcoding one target only by not adding a target_id identifier
// TODO(Jack): Add not in docs that we cannot cover all error conditions in unit test so we suppress the errors

namespace reprojection::database {

// RENAME - remove the data suffix
void AddExtractedTargetData(CameraMeasurement const& data, std::string_view sensor_name,
                            std::shared_ptr<CalibrationDatabase> const database);

void AddCameraPoseData(Frames const& data, std::string_view sensor_name, PoseType const type,
                       std::shared_ptr<CalibrationDatabase> const database);

// NOTE(Jack): We are violating the rule of passing in more information into a function than is required by passing in
// the entire CameraCalibrationData object. However here we pass it as a read only reference, unlike the calibration
// optimization related uses which reads some data and mutates some data. For those cases we use the "view" construct to
// slice the data and only expose the necessary parts. We could do that here too and make a AddPoseView, but that would
// add a lot of boilerplate code and not really move us forward to the end goal, or protect us from heinous abuses. It
// is already a const& so the only risk is that someone does too much with the extra non-pose data in this method. But
// if someone does that in a method named AddPoseData, then I think we have bigger problems :)
void AddPoseData(Frames const& data, std::string_view sensor_name, PoseType const type, std::string_view sql,
                 std::shared_ptr<CalibrationDatabase> const database);

void AddReprojectionError(std::map<uint64_t, ArrayX2d> const& data, std::string_view sensor_name, PoseType const type,
                          std::shared_ptr<CalibrationDatabase> const database);

[[nodiscard]] bool AddImuData(ImuMeasurement const& data, std::string_view sensor_name,
                              std::shared_ptr<CalibrationDatabase> const database);

}  // namespace reprojection::database