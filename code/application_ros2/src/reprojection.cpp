#include "application_ros2/reprojection.hpp"

#include <filesystem>
#include <iostream>

#include "image_loading.hpp"

namespace reprojection::ros2 {

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

std::optional<std::pair<uint64_t, cv::Mat>> ImageSource::operator()() {
    if (auto const msg{bag_reader.Next()}) {
        auto const data_i{ros2::ToCvMat(*msg, bag_reader.topic_type)};
        return data_i;
    }

    return std::nullopt;
}

}  // namespace reprojection::ros2
