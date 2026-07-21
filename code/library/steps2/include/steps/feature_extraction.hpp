#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct FeatureExtraction {
    std::string camera_name_;
    std::shared_ptr<EncodedImages> images_;
    TargetInfo target_info_;
    bool show_extraction_;

    static CalibrationStep StepType() { return CalibrationStep::FeatureExtraction; }

    std::string EntityId() const { return camera_name_; }

    std::string HashInputs() const;

    CameraMeasurements Compute() const;

    CameraMeasurements Load(SqlitePtr const db) const;

    void Save(CameraMeasurements const& extracted_targets, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
