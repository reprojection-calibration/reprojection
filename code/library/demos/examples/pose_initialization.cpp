#include <map>

#include "calibration/linear_pose_initialization.hpp"
#include "database/calibration_database.hpp"
#include "database/sensor_data_interface.hpp"
#include "geometry/lie.hpp"
#include "optimization/nonlinear_refinement.hpp"
#include "types/RENAME_AND_MOVE.hpp"

using namespace reprojection;

// WARN(Jack): At this time this demo has no clear role in CI/CD or the active development. Please feel to remove this
// as needed!

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, false, false)};

    CameraSensorData cam0{{"/cam0/image_raw", CameraModel::DoubleSphere},
                          Array6d{156.82590211, 156.79756958, 254.99978685, 256.9744566, -0.17931409, 0.59133716},
                          {},
                          {}};

    // Cam 0
    // TODO LOAD THE TARGET DIRECTLY INTO THE NEW DATA STRUCTURE
    auto const cam0_targets_stamped{
        database::GetExtractedTargetData(db, "/cam0/image_raw").value()};  // WARN UNPROTECTED OPTIONAL ACCESS
    for (auto const& target : cam0_targets_stamped) {
        cam0.frames[target.header.timestamp_ns].extracted_target = target.target;
    }

    calibration::LinearPoseInitialization(InitializationDataView(cam0));

    // TODO HANDLE POSE DATA AND DATABASE INTERACTIONS NATIVELY WITH A VIEW/THE NEW DATA STRUCT
    std::set<PoseStamped> cam0_poses;
    for (auto const& frame_i : cam0.frames) {
        cam0_poses.insert({{frame_i.first, cam0.sensor.sensor_name}, frame_i.second.initial_pose});
    }
    (void)AddPoseData(cam0_poses, database::PoseType::Initial, db);

    int del{0};
    std::vector<Frame> frames;
    for (auto const& frame_i : cam0.frames) {
        frames.push_back({frame_i.second.extracted_target.bundle, geometry::Exp(frame_i.second.initial_pose)});

        ++del;
        if (del > 400) {
            break;
        }
    }

    auto const [poses_opt, K, final_cost]{
        optimization::CameraNonlinearRefinement(frames, CameraModel::DoubleSphere, cam0.initial_intrinsics)};

    std::cout << K.transpose() << std::endl;

    std::set<PoseStamped> db_poses_opt;
    int i{0};
    for (auto const& header_i : cam0_targets_stamped) {
        db_poses_opt.insert({header_i.header, geometry::Log(poses_opt[i].inverse())});  // INVERSE???
        i++;
        if (i > 400) {
            break;
        }
    }
    (void)AddPoseData(db_poses_opt, database::PoseType::Optimized, db);

    return EXIT_SUCCESS;
}