#include <map>
#include <ranges>

#include "caching/cache_keys.hpp"
#include "calibration/initialization_methods.hpp"
#include "database/calibration_database.hpp"
#include "optimization/camera_imu_nonlinear_refinement.hpp"
#include "projection_functions/double_sphere.hpp"
#include "spline/se3_spline.hpp"
#include "spline/spline_initialization.hpp"
#include "steps/camera_info.hpp"
#include "steps/camera_nonlinear_refinement.hpp"
#include "steps/feature_extraction.hpp"
#include "steps/intrinsic_initialization.hpp"
#include "steps/linear_pose_initialization.hpp"
#include "steps/step_runner.hpp"

using namespace reprojection;

// WARN(Jack): At this time this demo has no clear role in CI/CD or the active development. Please feel to remove this
// as needed!

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

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, false, false)};

    static constexpr std::string_view config_file{R"(
            [sensor]
            camera_name = "/cam0/image_raw"
            camera_model = "double_sphere"

            [target]
            pattern_size = [3,4]
            type = "circle_grid"
        )"};
    toml::table const config{toml::parse(config_file)};

    try {
        CameraInfo const camera_info{config["sensor"]["camera_name"].as_string()->get(),
                                     ToCameraModel(config["sensor"]["camera_model"].as_string()->get()),
                                     {0, 512, 0, 512}};
        database::WriteToDb(camera_info, db);

        std::ostringstream oss1;
        oss1 << *config["sensor"].as_table();
        database::WriteToDb(CalibrationStep::CameraInfo, caching::CacheKey(oss1.str()), camera_info.sensor_name, db);

        std::ostringstream oss2;
        oss2 << *config["target"].as_table();
        database::WriteToDb(CalibrationStep::FtEx, caching::CacheKey(oss2.str()), camera_info.sensor_name, db);
    } catch (...) {
    }

    steps::CameraInfoStep const ci_step{"", *config["sensor"].as_table(), {}};
    auto const [camera_info, ci_cache_status]{steps::RunStep<CameraInfo>(ci_step, db)};
    std::cout << "Ci : " << ToString(ci_cache_status) << " " << camera_info.sensor_name << std::endl;

    steps::FeatureExtractionStep const ftext_step{camera_info.sensor_name, "", {}, *config["target"].as_table()};
    auto const [targets, ftext_cache_status]{steps::RunStep<CameraMeasurements>(ftext_step, db)};
    std::cout << "Ftext : " << ToString(ftext_cache_status) << " " << std::size(targets) << std::endl;

    steps::IntrinsicInitializationStep const ii_step{camera_info, targets};
    auto const [camera_state, ii_cache_status]{steps::RunStep<CameraState>(ii_step, db)};
    std::cout << "Ii : " << ToString(ii_cache_status) << " " << camera_state.intrinsics.transpose() << std::endl;

    steps::LpiStep const lpi_step{camera_info, targets, camera_state};
    auto const [initial_poses, lpi_cache_status]{steps::RunStep<Frames>(lpi_step, db)};
    std::cout << "Lpi : " << ToString(lpi_cache_status) << std::endl;

    auto const aligned_initial_state{AlignRotations({camera_state, initial_poses})};

    steps::CnlrStep const cnlr_step{camera_info, targets, aligned_initial_state};
    auto const [optimized_state, cnlr_cache_status]{steps::RunStep<OptimizationState>(cnlr_step, db)};
    std::cout << "Cnlr : " << ToString(cnlr_cache_status) << " " << optimized_state.camera_state.intrinsics.transpose()
              << std::endl;

    return EXIT_SUCCESS;
}
