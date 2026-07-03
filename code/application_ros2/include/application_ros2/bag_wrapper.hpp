#pragma once

#include <format>
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
    std::string topic_;
    std::string topic_type_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;

    static std::variant<SingleTopicBagReader, BagError> Create(std::string const& path, std::string const& topic) {
        try {
            auto reader{std::make_unique<rosbag2_cpp::Reader>()};

            reader->open(path);
            reader->set_filter(rosbag2_storage::StorageFilter{{topic}, {}});

            // NOTE(Jack): Technically there should only be one topic here I guess, but just to make sure ROS2 does not
            // surprise us we construct and then query the topic_type_map. It would be nice if we had an easier way to
            // interface with the serialized data, but as far as I can tell this is the best we get.
            std::map<std::string, std::string> topic_type_map;
            for (auto const& topic_i : reader->get_all_topics_and_types()) {
                topic_type_map[topic_i.name] = topic_i.type;
            }

            // TODO(Jack): We should really use the loggin functionality from the library.
            if (auto const it{topic_type_map.find(topic)}; it == std::cend(topic_type_map)) {
                return BagError{std::format("Bag {} does not contain requested topic {}", path, topic)};
            }

            return SingleTopicBagReader{topic, topic_type_map.at(topic), std::move(reader)};
        } catch (std::exception const& e) {
            return BagError{std::format("Bag {} loaded with error {}", path, e.what())};
        }
    }

    // TODO(Jack): Do we need to reset the reader after we run out? It seems like after we read through them all that
    //  we should reset back to the start so the next processing step can iterate over them.
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> Next() const {
        if (reader_->has_next()) {
            auto const msg{reader_->read_next()};

            return msg;
        }

        // NOTE(Jack): We want the capability to iterate through the images multiples times. However the ros2 bag reader
        // API provides us no explicit "rest back to start" functionality, therefore we instead the "seek" method to
        // bring us back to the start. If this is 100% foolproof I am still not sure, but it seems to work.
        reader_->seek(0);

        return nullptr;
    }

   private:
    // TODO(Jack): Clarify the move semantics here. There is likely no requirement for us to use move for the string
    // parameters right?
    SingleTopicBagReader(std::string_view topic, std::string_view topic_type,
                         std::unique_ptr<rosbag2_cpp::Reader> reader)
        : topic_{topic}, topic_type_{topic_type}, reader_{std::move(reader)} {}
};

}  // namespace reprojection::ros2