#include "application/calibrate.hpp"

#include <gtest/gtest.h>

#include <memory>

#include "caching/cache_keys.hpp"
#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

TEST(Application, TestCalibrate) {
    // TODO(Jack): This is now copy and pasted in a lot of places right? Should we do a central definition?
    static constexpr std::string_view config_file{R"(
            [sensor]
            camera_name = "/cam0/image_raw"
            camera_model = "double_sphere"

            [target]
            pattern_size = [3,4]
            type = "circle_grid"
        )"};
    toml::table const config{toml::parse(config_file)};

    auto db{database::OpenCalibrationDatabase(":memory:", true, false)};

    // Write the camera info
    // TODO(Jack): Add foreign key constraint that a camera info must have a step!
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

    database::WriteToDb(CalibrationStep::Ii, caching::CacheKey(camera_info, {}), camera_info.sensor_name, db);
    database::WriteToDb({Array6d::Zero()}, camera_info.camera_model, CalibrationStep::Ii, camera_info.sensor_name, db);

    EXPECT_NO_THROW(application::Calibrate(config, {}, "", db));
}
