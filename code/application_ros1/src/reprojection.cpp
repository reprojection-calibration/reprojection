#include "reprojection/reprojection.hpp"
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace reprojection::ros1 {

std::tuple<std::string, std::string> DummyLoadConfig() {
    return {"/data/TUM-Visual-Inertial-Dataset/dataset-calib-imu4.bag", "/cam0/image_raw"};
}

std::optional<std::string> CalculateCacheString(BagWrapper const& bag, std::string_view topic) {
    rosbag::View view(bag.bag, rosbag::TopicQuery({std::string(topic)}));

    if (view.size() == 0) {
        return std::nullopt;
    }



    for (auto const& msg : view) {
        std::cout << msg.getTime() << std::endl;
        std::cout << view.size() << std::endl;
    }

    return "";
}

}  // namespace reprojection::ros1
