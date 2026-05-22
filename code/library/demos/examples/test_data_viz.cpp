
#include <ranges>

#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "caching/cache_keys.hpp"
#include "config/config_parsing.hpp"
#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/test_data.db3"};
    auto db{database::OpenCalibrationDatabase(record_path, true, false)};

    static constexpr std::string_view config_file{R"(
            [sensor]
            camera_name = "test_sensor"
            camera_model = "double_sphere"

            [target]
            pattern_size = [6,6]
            type = "aprilgrid3"
            unit_dimension = 0.1
        )"};
    toml::table const config{toml::parse(config_file)};

    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::image_bounds};
    uint64_t const timespan_ns{10000000000};
    auto const [_, camera_frames]{
        testing_mocks::GenerateMvgData(sensor, CameraState{testing_utilities::pinhole_intrinsics}, 200, timespan_ns)};

    return EXIT_SUCCESS;
}
