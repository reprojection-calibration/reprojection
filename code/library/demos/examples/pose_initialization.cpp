
#include "application/calibrate.hpp"
#include "database/database_write.hpp"

using namespace reprojection;
using namespace std::string_view_literals;

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

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, false, false)};

    // Artificially trigger a cache hit by writing a feature extraction key and camera info to the database - the
    // features are already there.
    CameraInfo const camera_info{"/cam0/image_raw", CameraModel::DoubleSphere, {0, 512, 0, 512}};
    try {
        database::WriteToDb(camera_info, db);
        database::WriteToDb(CalibrationStep::FtEx, "ftex_key", camera_info.sensor_name, db);
    } catch (...) {
    }

    static constexpr std::string_view config_file{R"(
        [sensor]
        camera_name = "/cam0/image_raw"
        camera_model = "double_sphere"

        [target]
        pattern_size = [0,0]
        type = "xxxx"
    )"sv};
    toml::table const config{toml::parse(config_file)};

   auto const cache_status{ application::Calibrate(config, {}, "ftex_key", db)};
    for (auto const& [step, status]: cache_status) {
        std::cout << ToString(step) << " " << ToString(status) << std::endl;
    }

    return EXIT_SUCCESS;
}
