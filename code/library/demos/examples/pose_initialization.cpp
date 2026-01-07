#include "calibration/linear_pose_initialization.hpp"
#include "database/calibration_database.hpp"
#include "database/sensor_data_interface.hpp"
#include "geometry/lie.hpp"
#include "optimization/nonlinear_refinement.hpp"

namespace reprojection::calibration {

// for every camera pose, which is unique according to timestamp and sensor name, we will have a image, extracted
// target, initial pose, optimized pose, initial reprojection error and reprojection error after optimization

struct CalibrationFrameKey {
    std::int64_t timestamp;
    std::string sensor_name;

    bool operator==(CalibrationFrameKey const& other) const {
        return timestamp == other.timestamp and sensor_name == other.sensor_name;
    }
};

struct CalibrationFrame {
    ExtractedTarget extracted_target;
    Vector6d initial_pose;
    Vector6d optimized_pose;
};

}  // namespace reprojection::calibration

using namespace reprojection;

// WARN(Jack): At this time this demo has no clear role in CI/CD or the active development. Please feel to remove this
// as needed!

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, false, false)};
    Array6d const cam0_ds_intrinsics{156.82590211, 156.79756958, 254.99978685, 256.9744566, -0.17931409, 0.59133716};

    // Cam 0
    auto const cam0_targets_stamped{database::GetExtractedTargetData(db, "/cam0/image_raw")};
    auto const cam0_poses{calibration::LinearPoseInitialization(
        cam0_targets_stamped.value(),
        std::unique_ptr<projection_functions::Camera>(
            new projection_functions::DoubleSphereCamera(cam0_ds_intrinsics)))};  // WARN UNPROTECTED OPTIONAL ACCESS
    (void)AddPoseData(cam0_poses, database::PoseTable::Camera, database::PoseType::Initial, db);

    // ERROR COUNTS ON DATA BEING PERFECTlY SYNCED AND NO FRAMES DROPPING OUT FOR ALIGNMENT!
    auto it1 = cam0_targets_stamped.value().begin();
    auto it2 = cam0_poses.begin();

    std::cout << std::size(cam0_targets_stamped.value()) << std::endl;
    std::cout << std::size(cam0_poses) << std::endl;

    std::vector<Frame> frames;
    int del{0};
    while (it1 != cam0_targets_stamped.value().end() && it2 != cam0_poses.end()) {
        frames.push_back({it1->target.bundle, geometry::Exp(it2->pose)});  // INVERSE???

        ++it1;
        ++it2;
        ++del;

        if (del > 400) {
            break;
        }
    }
    std::cout << "got all frames" << std::endl;

    auto const [poses_opt, K, final_cost]{
        optimization::CameraNonlinearRefinement(frames, CameraModel::DoubleSphere, cam0_ds_intrinsics)};

    std::cout << K.transpose() << std::endl;

    std::set<PoseStamped> db_poses_opt;
    int i{0};
    for (auto const& header_i : cam0_targets_stamped.value()) {
        db_poses_opt.insert({header_i.header, geometry::Log(poses_opt[i].inverse())});  // INVERSE???
        i++;
        if (i > 400) {
            break;
        }
    }
    (void)AddPoseData(db_poses_opt, database::PoseTable::Camera, database::PoseType::Optimized, db);

    return EXIT_SUCCESS;
}