#pragma once

#include <gtest/gtest.h>

#include <string>

#include "database/calibration_database.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

// TODO(Jack): There is no real reason for these test fxitures to be in their own file. Should be commbined with the
// tests directly.

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
using ImuErrors = reprojection::ImuErrors;
using ImuMeasurements = reprojection::ImuMeasurements;
using MatrixX2d = reprojection::MatrixX2d;
using MatrixX3d = reprojection::MatrixX3d;
using SqlitePtr = reprojection::SqlitePtr;
using TargetInfo = reprojection::TargetInfo;
using TargetType = reprojection::TargetType;

class CameraDatabaseFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = db::OpenCalibrationDatabase(":memory:", true, false);

        db::InsertEntity(db, sensor_name, Entity::Camera);
    }

    void InsertCameraInfo() const {
        // NOTE(Jack): The camera info, target info, image loading, and extracted target tables are unique because they
        // can only be associated with one specific step each. That is the reason why for these three Add*() methods we
        // add the correspondent step directly. For other values we let the caller decide which step they want to add
        // depending on the context.
        InsertStep(CalibrationStep::CameraInfo);

        db::InsertCameraInfo(db, camera_info);
    }

    void InsertImage() const {
        // NOTE(Jack): See note in InsertCameraInfo() above.
        InsertStep(CalibrationStep::ImageLoading);

        db::InsertImages(db, sensor_name, EncodedImages{{timestamp_ns, {}}});
    }

    void InsertIntrinsic(CalibrationStep const step_type) const {
        db::InsertIntrinsics(db, sensor_name, step_type, CameraModel::Pinhole, {tu::pinhole_intrinsics});
    }

    void InsertStep(CalibrationStep const step_type, std::string const& cache_key = "") const {
        db::InsertStep(db, sensor_name, step_type, cache_key);
    }

    void InsertTarget() const {
        // NOTE(Jack): See note in InsertCameraInfo() above.
        InsertStep(CalibrationStep::FeatureExtraction);

        db::InsertTargets(db, sensor_name, {{timestamp_ns, target}});
    }

    void InsertTargetInfo() const {
        // NOTE(Jack): See note in InsertCameraInfo() above.
        InsertStep(CalibrationStep::TargetInfo);

        db::InsertTargetInfo(db, sensor_name, target_info);
    }

    void InsertPose(CalibrationStep const step_name) const {
        Frames const frames{{timestamp_ns, {pose}}};

        db::InsertPoses(db, sensor_name, step_name, frames);
    }

    SqlitePtr db{nullptr};
    uint64_t timestamp_ns{0};
    std::string sensor_name{"/cam/retro/123"};

    CameraInfo camera_info{sensor_name, CameraModel::Pinhole, tu::image_bounds};
    Array6d pose{0, 1, 2, 3, 4, 5};
    ExtractedTarget target{{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}, {200.24, 300.56}},
                            MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}, {300.65, 200.56, 712.57}}},
                           {{5, 6}, {2, 3}, {650, 600}}};
    TargetInfo target_info{TargetType::Aprilgrid3, 8, 6, 0.1, false};

    // All the data values - we store these as part of the fixture so we can compare the reread values to the
    // groundtruth stored here.
};

class ImuDatabaseFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = db::OpenCalibrationDatabase(":memory:", true, false);

        db::InsertEntity(db, sensor_name, Entity::Imu);
    }

    void InsertStep(CalibrationStep const step_name, std::string const& cache_key = "") const {
        db::InsertStep(db, sensor_name, step_name, cache_key);
    }

    void InsertImuData() const { db::InsertImuData(db, sensor_name, imu_data); }

    void InsertImuError(CalibrationStep const step_type) const {
        db::InsertImuErrors(db, sensor_name, step_type, imu_errors);
    }

    SqlitePtr db{nullptr};
    uint64_t timestamp_ns{0};
    std::string sensor_name{"/imu/polaris/123"};

    ImuMeasurements imu_data{{timestamp_ns, {{1, 2, 3}, {4, 5, 6}}}};
    ImuErrors imu_errors{{timestamp_ns, {{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}}}};
};

class ExtrinsicDatabaseFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = db::OpenCalibrationDatabase(":memory:", true, false);

        db::InsertEntity(db, imu_name, Entity::Imu);
        db::InsertEntity(db, camera_name, Entity::Camera);
        db::InsertEntity(db, extrinsic_id, Entity::Extrinsic);
    }

    void InsertStep(std::string_view entity_id, CalibrationStep const step_name,
                    std::string const& cache_key = "") const {
        db::InsertStep(db, entity_id, step_name, cache_key);
    }

    SqlitePtr db{nullptr};
    std::string imu_name{"/imu/polaris/123"};
    std::string camera_name{"/cam/retro/123"};
    // TODO(Jack): This construction of the extrinsic id key is now replicated in at least three different places - copy
    // and pasted.
    std::string extrinsic_id{"tf_" + imu_name + "_xxx_" + camera_name};
};