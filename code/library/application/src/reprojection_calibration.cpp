#include "application/reprojection_calibration.hpp"

#include <ranges>

#include "step_runner.hpp"
#include "steps.hpp"

namespace reprojection::application {

// WARN(Jack): This is a hack that we need to do so that the spline initialization does not have any massive
// discontinuities or sudden jumps. But there is some bigger problem here that we are missing and need to solve long
// term.
// WARN(Jack): Also note that we do not save aligned_initial_state to the database, we save plain old initial_state and
// use that to calculate the reprojection errors, but use aligned_initial_state to initialize the nonlinear
// optimization. This means that what we are doing here and what we are visualizing in the database are starting to
// diverge. Not nice!
// cppcheck-suppress passedByValue
OptimizationState AlignRotations(OptimizationState state) {
    Vector3d so3_i_1{std::cbegin(state.frames)->second.pose.head<3>()};
    for (auto& frame_i : state.frames | std::views::values) {
        Vector3d so3_i{frame_i.pose.head<3>()};
        double const dp{so3_i_1.dot(so3_i)};

        if (dp < 0) {
            so3_i *= -1.0;
        }
        frame_i.pose.head<3>() = so3_i;

        so3_i_1 = so3_i;
    }

    return state;
}

bool CameraCalibration(std::string const& db_path) {
    auto db{std::make_shared<database::CalibrationDatabase>(db_path, false, false)};

    CameraInfo const camera_info{"/cam0/image_raw", CameraModel::DoubleSphere, {0, 512, 0, 512}};
    try {
        database::WriteToDb(camera_info, db);
    } catch (...) {
    }

    // Load targets, initialize, and optimize
    CameraMeasurements const targets{database::GetExtractedTargetData(db, camera_info.sensor_name)};

    application::IiStep const ii_step{camera_info, targets};
    auto const [camera_state, ii_cache_status]{application::RunStep<CameraState>(ii_step, db)};
    std::cout << "Ii : " << ToString(ii_cache_status) << " " << camera_state.intrinsics.transpose() << std::endl;

    application::LpiStep const lpi_step{camera_info, targets, camera_state};
    auto const [initial_poses, lpi_cache_status]{application::RunStep<Frames>(lpi_step, db)};
    std::cout << "Lpi : " << ToString(lpi_cache_status) << std::endl;

    auto const aligned_initial_state{AlignRotations({camera_state, initial_poses})};

    application::CnlrStep const cnlr_step{camera_info, targets, aligned_initial_state};
    auto const [optimized_state, cnlr_cache_status]{application::RunStep<OptimizationState>(cnlr_step, db)};
    std::cout << "Cnlr : " << ToString(cnlr_cache_status) << " " << optimized_state.camera_state.intrinsics.transpose()
              << std::endl;

    return true;
}

}  // namespace reprojection::application