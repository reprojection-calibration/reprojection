
#include <ranges>

#include <toml++/toml.hpp>

#include "application/calibrate.hpp"
#include "caching/cache_keys.hpp"
#include "database/calibration_database.hpp"
#include "database/database_write.hpp"

using namespace reprojection;

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

    application::Calibrate(config, {}, "", db);

    return EXIT_SUCCESS;
}
