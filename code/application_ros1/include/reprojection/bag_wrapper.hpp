#pragma once

#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>

#include <string>

namespace reprojection::ros1 {

struct BagWrapper {
    BagWrapper(std::string const& path, rosbag::BagMode const mode) { bag.open(path, mode); }

    ~BagWrapper() { bag.close(); }

    rosbag::Bag bag;
};

struct SingleTopicBagReader {
    SingleTopicBagReader(std::string const& path, std::string const& _topic)
        : topic{_topic},
          bag{BagWrapper(path, rosbag::bagmode::Read)},
          view{rosbag::View(bag.bag, rosbag::TopicQuery({topic}))} {}

    std::string topic;
    BagWrapper bag;
    rosbag::View view;
};

}  // namespace reprojection::ros1