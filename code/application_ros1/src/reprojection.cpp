#include "reprojection/reprojection.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <filesystem>

namespace reprojection::ros1 {

std::tuple<std::string, std::string> DummyLoadConfig() {
    return {"/data/TUM-Visual-Inertial-Dataset/dataset-calib-imu4.bag", "/cam0/image_raw"};
}

std::optional<std::string> TopicCacheString(BagWrapper const& bag, std::string_view topic) {
    if (bag.bag.getMode() != rosbag::BagMode::Read) {
        return std::nullopt;
    }

    rosbag::View view(bag.bag, rosbag::TopicQuery({std::string(topic)}));
    if (view.size() == 0) {
        return std::nullopt;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    oss << std::filesystem::path(bag.bag.getFileName()).filename().c_str() << "|";
    oss << topic << "|";
    oss << std::to_string(view.size()) << "|";

    for (auto const& msg : view) {
        oss << msg.getTime().toSec() << ";";
    }
    oss << "|";

    return oss.str();
}

}  // namespace reprojection::ros1
