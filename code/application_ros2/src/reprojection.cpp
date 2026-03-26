#include "reprojection/reprojection.hpp"

#include <filesystem>
#include <iostream>

#include <rosbag2_cpp/reader.hpp>

namespace reprojection::ros2 {

// NOTE(Jack): The entire points of the SerializeBagTopic function here and in the ROS1 app is for us to calculate a
// unique signature of the image data WITHOUT having to deserialize the data! I think that would cost way too much CPU,
// but I also never benchmarked it :)
std::optional<std::string> SerializeBagTopic(SingleTopicBagReader const& data) {
    std::ostringstream oss;
    oss << std::filesystem::path(data.reader->get_metadata().files[0].path).filename().c_str() << "|";
    oss << data.topic << "|";

    int count{0};
    while (auto const msg{data.Next()}) {
        //  NOTE(Jack): This returns a long int representing the time in nanoseconds. In ROS 1 we just get a float.
        oss << msg->recv_timestamp << ";";

        count++;
    }
    oss << "|";

    // NOTE(Jack): We want to have the same semantics as the ROS1 case where if there are no messages in the topic we
    // return std::nullopt. But ROS2 does not provide us a way to query this directly so we have to count manually in
    // the loop and then check.
    if (count == 0) {
        return std::nullopt;
    }

    return oss.str();
}

}  // namespace reprojection::ros2
