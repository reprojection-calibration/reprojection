#pragma once

#include <gtest/gtest.h>

#include <string>

#include "database/calibration_database.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

// NOTE(Jack): This is a little ridiculous to use all these declarations here, but we are in a header file and I did not
// want to do something like "using namespace reprojection;" here because that kind of namespace exposure in a header
// file can be confusing later.
namespace db = reprojection::database;
namespace tu = reprojection::testing_utilities;

using Array6d = reprojection::Array6d;
using CameraInfo = reprojection::CameraInfo;
using CameraModel = reprojection::CameraModel;
using CalibrationStep = reprojection::CalibrationStep;
using EncodedImages = reprojection::EncodedImages;
using Entity = reprojection::Entity;
using ExtractedTarget = reprojection::ExtractedTarget;
using Frames = reprojection::Frames;
using SqlitePtr = reprojection::SqlitePtr;
using TargetInfo = reprojection::TargetInfo;
using TargetType = reprojection::TargetType;

class CameraDatabaseFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = db::OpenCalibrationDatabase(":memory:", true, false);

        db::InsertEntity(db, sensor_name, Entity::Camera);
    }

    void InsertStep(CalibrationStep const step_name, std::string const& cache_key = "") const {
        db::InsertStep(db, sensor_name, step_name, cache_key);
    }

    void InsertCameraInfo() const {
        // NOTE(Jack): The camera info, target info, image loading, and extracted target tables are unique because they
        // can only be associated with one specific step each. That is the reason why for these three Add*() methods we
        // add the correspondent step directly. For other values we let the caller decide which step they want to add
        // depending on the context.
        InsertStep(CalibrationStep::CameraInfo);

        db::InsertCameraInfo(db, CameraInfo{sensor_name, CameraModel::Pinhole, tu::image_bounds});
    }

    void InsertImage() const {
        // NOTE(Jack): See note in InsertCameraInfo() above.
        InsertStep(CalibrationStep::ImageLoading);

        db::InsertImages(db, sensor_name, EncodedImages{{timestamp_ns, {}}});
    }

    void InsertTarget() const {
        // NOTE(Jack): See note in InsertCameraInfo() above.
        InsertStep(CalibrationStep::FeatureExtraction);

        db::InsertTargets(db, sensor_name, {{timestamp_ns, ExtractedTarget{{{}, {}}, {}}}});
    }

    void InsertTargetInfo() const {
        // NOTE(Jack): See note in InsertCameraInfo() above.
        InsertStep(CalibrationStep::TargetInfo);

        db::InsertTargetInfo(db, sensor_name, target_info);
    }

    void InsertPose(CalibrationStep const step_name) const {
        Frames const frames{{timestamp_ns, {Array6d{0, 1, 2, 3, 4, 5}}}};
        db::InsertPoses(db, sensor_name, step_name, frames);
    }

    SqlitePtr db{nullptr};
    uint64_t timestamp_ns{0};
    std::string sensor_name{"/cam/retro/123"};
    TargetInfo target_info { TargetType::Aprilgrid3, 8, 6, 0.1, false };

    // All the data values - we store these as part of the fixture so we can compare the reread values to the
    // groundtruth stored here.
};
