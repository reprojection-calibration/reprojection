#pragma once

#include <rosbag/bag.h>

#include <string>

namespace reprojection::ros1 {

struct BagWrapper {
    BagWrapper(std::string const& path, rosbag::BagMode const mode) { bag.open(path, mode); }

    ~BagWrapper() { bag.close(); }

    rosbag::Bag bag;
};

}  // namespace reprojection::ros1