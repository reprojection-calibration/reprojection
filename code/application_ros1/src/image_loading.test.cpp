#include "reprojection/image_loading.hpp"

#include <gtest/gtest.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "testing_utils.hpp"

using namespace reprojection;

TEST(Ros1ImageLoading, TestRawAndCompressedToCvMat) {
    // NOTE(Jack): Our ros1::ToCvMat() function works directly on the rosbag::MessageInstance. This type, as far as I
    // understand, cannot really be created as normal standalone objects. The only place that you can really get them is
    // by iterating a open bag via a rosbag::View. Therefore, in this test we need to write a bag and then process it to
    // get the test data in the required format.
    ros1::ScopedBagPath const temp_bag;

    rosbag::Bag bag;
    bag.open(temp_bag.path, rosbag::bagmode::Write);
    bag.write("/raw_image_topic", ros::Time(1), ros1::DummyImage());
    bag.write("/compressed_image_topic", ros::Time(1), ros1::DummyCompressedImage());
    bag.close();

    bag.open(temp_bag.path, rosbag::bagmode::Read);
    rosbag::View view(bag);
    ASSERT_EQ(view.size(), 2);

    for (auto const& msg : view) {
        std::pair<uint64_t, cv::Mat> result;
        EXPECT_NO_THROW(result = ros1::ToCvMat(msg));

        auto const& [timestamp_ns, img]{result};
        EXPECT_EQ(timestamp_ns, 1e9);
        EXPECT_EQ(img.rows, 1);
        EXPECT_EQ(img.cols, 1);
    }

    bag.close();
}