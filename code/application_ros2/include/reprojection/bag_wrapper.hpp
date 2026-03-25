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
    std::unique_ptr<rosbag2_cpp::Reader> reader;

    static std::variant<SingleTopicBagReader, BagError> Create(std::string const& path, std::string const& topic) {
        try {
            auto reader{std::make_unique<rosbag2_cpp::Reader>()};

            reader->open(path);
            reader->set_filter(rosbag2_storage::StorageFilter{{topic}});

            return SingleTopicBagReader{topic, std::move(reader)};
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
    SingleTopicBagReader(std::string _topic, std::unique_ptr<rosbag2_cpp::Reader> _reader)
        : topic(std::move(_topic)), reader(std::move(_reader)) {}
};

}  // namespace reprojection::ros2