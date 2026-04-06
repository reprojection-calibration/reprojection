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

// TODO(Jack): Should we refactor to use optional and shared pointer instead if variant and mutable references/value
// semantics?
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
            // surprise us we construct and then query the topic_type_map. It would be nice if we had an easier way to
            // interface with the serialized data, but as far as I can tell this is the best we get.
            std::map<std::string, std::string> topic_type_map;
            for (auto const& topic_i : reader->get_all_topics_and_types()) {
                topic_type_map[topic_i.name] = topic_i.type;
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

        // NOTE(Jack): We want the capability to iterate through the images multiples times. However the ros2 bag reader
        // API provides us no explicit "rest back to start" functionality, therefore we instead the "seek" method to
        // bring us back to the start. If this is 100% foolproof I am still not sure, but it seems to work.
        reader->seek(0);

        return nullptr;
    }

   private:
    // TODO(Jack): Clarify the move semantics here. There is likely no requirement for us to use move for the string
    // parameters right?
    SingleTopicBagReader(std::string _topic, std::string _topic_type, std::unique_ptr<rosbag2_cpp::Reader> _reader)
        : topic(std::move(_topic)), topic_type(std::move(_topic_type)), reader(std::move(_reader)) {}
};

}  // namespace reprojection::ros2