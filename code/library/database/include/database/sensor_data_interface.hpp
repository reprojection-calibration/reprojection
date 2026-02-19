#pragma once

#include <memory>
#include <optional>
#include <set>
#include <string>

#include <opencv2/opencv.hpp>

#include "database/calibration_database.hpp"
#include "database/database_data_types.hpp"
#include "types/calibration_types.hpp"
#include "types/sensor_types.hpp"

// TOP LEVEL DESIGN NOTE
// We are at an in between point now (09.01.2026) where we have begun to migrate away from "stamped" data and individual
// "sets" of sensor data. We have instead moved towards the more centralized CameraCalibrationData representation, but
// have not yet transitioned all database funtions to work with that representations yet.
//
// Furthermore, we also began to transition away from std::optional and boolean flags towards a more strict policy of
// failure will be treated with a throw. We should also adopt this policy for all sensor data methods.

// TODO(Jack): Add note in docs that we are hardcoding one target only by not adding a target_id identifier
// TODO(Jack): Add not in docs that we cannot cover all error conditions in unit test so we suppress the errors
// NOTE(Jack): We need this streaming interface here because it is not feasible to load all images at once into memory,
// we will run into problems here with memory. Therefore, we create this streamer class which loads the images one by
// one from the database. We include the start_time parameter so that applications can skip loading images prior to that
// timestamp (i.e. if the images before start_time were already processed).
// WARN(Jack): We need to codify how we deal with const with the database. At this point I think there is no clear
// strategy or we might think we have protection where we do not really.

namespace reprojection::database {

void AddCameraPoseData(Frames const& data, PoseType const type, std::string_view sensor_name,
                       std::shared_ptr<CalibrationDatabase> const database);

// TODO(Jack): We need to figure out an overarching strategy of how to unify the camera and spline types. For example
//  both have initial and optimized poses! There has to be some commonality here we can take advantage of.
using SplinePoses = std::map<std::uint64_t, Vector6d>;

void AddSplinePoseData(SplinePoses const& data, PoseType const type, std::string_view sensor_name,
                       std::shared_ptr<CalibrationDatabase> const database);

// NOTE(Jack): We are violating the rule of passing in more information into a function than is required by passing in
// the entire CameraCalibrationData object. However here we pass it as a read only reference, unlike the calibration
// optimization related uses which reads some data and mutates some data. For those cases we use the "view" construct to
// slice the data and only expose the necessary parts. We could do that here too and make a AddPoseView, but that would
// add a lot of boilerplate code and not really move us forward to the end goal, or protect us from heinous abuses. It
// is already a const& so the only risk is that someone does too much with the extra non-pose data in this method. But
// if someone does that in a method named AddPoseData, then I think we have bigger problems :)
void AddPoseData(std::string_view const sql, Frames const& data, PoseType const type, std::string_view sensor_name,
                 std::shared_ptr<CalibrationDatabase> const database);

void AddReprojectionError(std::map<uint64_t, ArrayX2d> const& data, PoseType const type, std::string_view sensor_name,
                          std::shared_ptr<CalibrationDatabase> const database);

// RENAME - remove the data suffix
void AddExtractedTargetData(CameraMeasurement const& data, std::string_view sensor_name,
                            std::shared_ptr<CalibrationDatabase> const database);

// See the note above AddPoseData. Here we are mutating the data, should we use a controlled view here?
CameraMeasurements GetExtractedTargetData(std::shared_ptr<CalibrationDatabase const> const database,
                                          std::string_view sensor_name);

[[nodiscard]] bool AddImuData(ImuMeasurement const& data, std::string_view sensor_name,
                              std::shared_ptr<CalibrationDatabase> const database);

ImuMeasurements GetImuData(std::shared_ptr<CalibrationDatabase const> const database, std::string_view sensor_name);

}  // namespace reprojection::database