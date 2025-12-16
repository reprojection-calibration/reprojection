#pragma once

#include <cstdint>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>
#include <set>
#include <string>

#include "database/calibration_database.hpp"
#include "database/database_data_types.hpp"

// TODO(Jack): Add note in docs that we are hardcoding one target only by not adding target_id
// TODO(Jack): Add not in docs that we cannot cover all error conditions in unit test so we suppress the errors
// NOTE(Jack): We need this streaming interface here because it is not feasible to load all images at once into memory,
// we will run into problems here with memory. Therefore, we create this streamer class which loads the images one by
// one from the database. We include the start_time parameter so that applications can skip loading images prior to that
// timestamp (i.e. if the images before start_time were already processed).
// WARN(Jack): We need to codify how we deal with const with the database. At this point I think there is no clear
// strategy or we might think we have protection where we do not really.

namespace reprojection::database {

[[nodiscard]] bool AddFrame(FrameHeader const& data, std::shared_ptr<CalibrationDatabase> const database);

[[nodiscard]] bool AddCameraPoseData(std::set<PoseStamped> const& data, PoseType const type,
                                     std::shared_ptr<CalibrationDatabase> const database);

[[nodiscard]] bool AddExtractedTargetData(ExtractedTargetStamped const& data,
                                          std::shared_ptr<CalibrationDatabase> const database);

std::optional<std::set<ExtractedTargetStamped>> GetExtractedTargetData(
    std::shared_ptr<CalibrationDatabase const> const database, std::string const& sensor_name);

[[nodiscard]] bool AddImuData(ImuStamped const& data, std::shared_ptr<CalibrationDatabase> const database);

std::optional<std::set<ImuStamped>> GetImuData(std::shared_ptr<CalibrationDatabase const> const database,
                                               std::string const& sensor_name);

}  // namespace reprojection::database