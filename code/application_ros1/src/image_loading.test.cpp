#include "image_loading.hpp"

#include <gtest/gtest.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "reprojection/bag_wrapper.hpp"

#include "testing_utils.hpp"

using namespace reprojection;

TEST(Ros1ImageLoading, TestRawAndCompressedToCvMat) {
    // NOTE(Jack): Our ros1::ToCvMat() function works directly on the rosbag::MessageInstance. This type, as far as I
    // understand, cannot really be created as normal standalone objects. The only place that you can really get them is
    // by iterating a open bag via a rosbag::View. Therefore, in this test we need to write a bag and then process it to
    // get the test data in the required format.
    ros1::ScopedBagPath const temp_bag;
    {
        ros1::BagWrapper handle(temp_bag.path, rosbag::bagmode::Write);
        handle.bag.write("/raw_image_topic", ros::Time(1), ros1::DummyImage());
        handle.bag.write("/compressed_image_topic", ros::Time(1), ros1::DummyCompressedImage());
    }

    ros1::BagWrapper handle(temp_bag.path, rosbag::bagmode::Read);
    rosbag::View view(handle.bag);

    ASSERT_EQ(view.size(), 2);
    for (auto const& msg : view) {
        cv::Mat image;
        EXPECT_NO_THROW(image = ros1::ToCvMat(msg));

        EXPECT_EQ(image.rows, 1);
        EXPECT_EQ(image.cols, 1);
    }
}