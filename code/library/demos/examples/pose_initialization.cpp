#include "calibration/linear_pose_initialization.hpp"
#include "database/calibration_database.hpp"
#include "database/sensor_data_interface.hpp"

using namespace reprojection;

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, false, false)};
    Array6d const cam0_ds_intrinsics{156.82590211, 156.79756958, 254.99978685, 256.9744566, -0.17931409, 0.59133716};



    // Cam 0
    auto const cam0_data{database::GetExtractedTargetData(db, "/cam0/image_raw")};

    // WARN UNPROTECTED OPTIONAL ACCESS
    auto const cam0_poses{calibration::LinearPoseInitialization(
        cam0_data.value(), std::unique_ptr<projection_functions::Camera>(
                               new projection_functions::DoubleSphereCamera(cam0_ds_intrinsics)))};
    (void)AddPoseData(cam0_poses, database::PoseTable::Camera, database::PoseType::Initial, db);

    // Cam 1 - uses cam0 intrinsics
    auto const cam1_data{database::GetExtractedTargetData(db, "/cam1/image_raw")};
    // WARN UNPROTECTED OPTIONAL ACCESS
    auto const cam1_poses{calibration::LinearPoseInitialization(
        cam1_data.value(), std::unique_ptr<projection_functions::Camera>(
                               new projection_functions::DoubleSphereCamera(cam0_ds_intrinsics)))};
    (void)AddPoseData(cam1_poses, database::PoseTable::Camera, database::PoseType::Initial, db);

    return EXIT_SUCCESS;
}