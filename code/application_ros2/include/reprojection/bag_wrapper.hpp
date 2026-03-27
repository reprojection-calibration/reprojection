#pragma once

#include <memory>
#include <string>
#include <variant>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace reprojection::ros2 {

struct BagError {
    std::string message;
};

struct SingleTopicBagReader {
    std::string topic;
    std::string topic_type;
    std::unique_ptr<rosbag2_cpp::Reader> reader;

    static std::variant<SingleTopicBagReader, BagError> Create(std::string const& path, std::string const& topic) {
        try {
            auto reader{std::make_unique<rosbag2_cpp::Reader>()};

            reader->open(path);
            reader->set_filter(rosbag2_storage::StorageFilter{{topic}});

            // NOTE(Jack): Technically there should only be one topic here I guess, but just to make sure ROS2 does not
            // surpise us we construct and then query the topic_type_map.
            std::map<std::string, std::string> topic_type_map;
            for (auto const& topic : reader->get_all_topics_and_types()) {
                topic_type_map[topic.name] = topic.type;
            }

            return SingleTopicBagReader{topic, topic_type_map.at(topic), std::move(reader)};
        } catch (...) {
            return BagError{"Error loading data: " + path};
        }
    }

    // TODO(Jack): Do we need to reset the reader after we run out? It seems like after we read through them all that
    //  we should reset back to the start so the next processing step can iterate over them.
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> Next() const {
        if (reader->has_next()) {
            auto const msg{reader->read_next()};

            return msg;
        }

        return nullptr;
    }

   private:
    SingleTopicBagReader(std::string _topic, std::string _topic_type, std::unique_ptr<rosbag2_cpp::Reader> _reader)
        : topic(std::move(_topic)), topic_type(std::move(_topic_type)), reader(std::move(_reader)) {}
};

}  // namespace reprojection::ros2