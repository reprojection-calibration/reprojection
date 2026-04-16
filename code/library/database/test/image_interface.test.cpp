#include "database/image_interface.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include <opencv2/opencv.hpp>

#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
#include "testing_utilities/constants.hpp"
#include "types/enums.hpp"

// TODO MAKE SURE TABLE IN TEST_DATA DB GET UPDATE WITH THE NEW MEASUREMENT FOREIGN CONSTRAINT TABLES!!!

using namespace reprojection;

class ImageInterfaceFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = database::OpenCalibrationDatabase(":memory:", true, false);

        database::WriteToDb(CameraInfo{sensor_name, CameraModel::Pinhole, testing_utilities::image_bounds}, db);
    }

    void AddImage(int const timestamp_ns) const { database::AddImage({timestamp_ns, image}, sensor_name, db); }

    SqlitePtr db{nullptr};
    std::string sensor_name{"/cam/retro/123"};
    cv::Mat image{cv::Mat(10, 20, CV_8UC1)};
};

TEST_F(ImageInterfaceFixture, TestAddImage) { EXPECT_NO_THROW(AddImage(0)); }

TEST_F(ImageInterfaceFixture, TestAddImageHeaderOnly) { EXPECT_NO_THROW(database::AddImage(0, sensor_name, db)); }
