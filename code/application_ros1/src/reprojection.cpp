#include "reprojection/reprojection.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <filesystem>

namespace reprojection::ros1 {

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

    // NOTE(Jack): Underlying our use of this function is the assumption that the bag name, topic name, and the
    // collection of all message timestamps is sufficient to uniquely identify a data stream. Sounds logical right? I
    // think so.
    oss << std::filesystem::path(bag.bag.getFileName()).filename().c_str() << "|";
    oss << topic << "|";

    for (auto const& msg : view) {
        oss << msg.getTime().toSec() << ";";
    }
    oss << "|";

    return oss.str();
}

}  // namespace reprojection::ros1
