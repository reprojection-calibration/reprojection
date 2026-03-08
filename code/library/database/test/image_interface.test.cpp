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
        db = std::make_shared<database::CalibrationDatabase>(":memory:", true, false);

        database::WriteToDb(CameraInfo{sensor_name, CameraModel::Pinhole, testing_utilities::image_bounds}, db);
    }

    void AddImage(int const timestamp_ns) const { database::AddImage({timestamp_ns, image}, sensor_name, db); }

    std::shared_ptr<database::CalibrationDatabase> db;
    std::string sensor_name{"/cam/retro/123"};
    cv::Mat image{cv::Mat(10, 20, CV_8UC1)};
};

TEST_F(ImageInterfaceFixture, TestAddImage) { EXPECT_NO_THROW(AddImage(0)); }

TEST_F(ImageInterfaceFixture, TestAddImageHeaderOnly) { EXPECT_NO_THROW(database::AddImage(0, sensor_name, db)); }

TEST_F(ImageInterfaceFixture, TestImageStreamer) {
    AddImage(0);
    AddImage(2);
    AddImage(4);

    database::ImageStreamer streamer{db, sensor_name};
    auto const frame_0{streamer.Next()};
    ASSERT_TRUE(frame_0.has_value());

    auto const& [timestamp_ns, loaded_image]{frame_0.value()};
    EXPECT_EQ(timestamp_ns, 0);
    EXPECT_EQ(loaded_image.rows, 10);
    EXPECT_EQ(loaded_image.cols, 20);

    EXPECT_TRUE(streamer.Next().has_value());  // Frame 2
    EXPECT_TRUE(streamer.Next().has_value());  // Frame 3, last frame
    EXPECT_FALSE(streamer.Next().has_value());
}

// This simulates the case where we have already processed the images up to a certain time and only want to load the
// images after that point.
TEST_F(ImageInterfaceFixture, TestImageStreamerStartTime) {
    AddImage(0);
    AddImage(2);
    AddImage(4);

    database::ImageStreamer streamer{db, sensor_name, 1};

    auto const frame_0{streamer.Next()};
    ASSERT_TRUE(frame_0.has_value());
    EXPECT_EQ(frame_0.value().first, 2);  // First frame starts at 2ns now

    EXPECT_TRUE(streamer.Next().has_value());
    EXPECT_FALSE(streamer.Next().has_value());
}
