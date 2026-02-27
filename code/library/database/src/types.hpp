#pragma once
#include <cstdint>
#include <optional>
#include <string>

struct DataKey {
    DataKey(std::string_view _step_name, std::string_view _sensor_name, std::uint64_t _timestamp_ns)
        : step_name{_step_name}, sensor_name{_sensor_name}, timestamp_ns{_timestamp_ns} {}

    DataKey(std::string_view _sensor_name, std::uint64_t _timestamp_ns)
        : sensor_name{_sensor_name}, timestamp_ns{_timestamp_ns} {}

    // NOTE(Jack): The step_name is optional because some data (usually measurements) might not have a "processing
    // step". For example, it would be redundant to give image data or extracted features a step name because the step
    // name is already coded into their existence.
    std::optional<std::string> step_name;
    std::string sensor_name;
    std::uint64_t timestamp_ns;
};