#pragma once

#include <sqlite3.h>

#include <cstdint>
#include <opencv2/opencv.hpp>
#include <optional>
#include <set>
#include <string>

namespace reprojection::database {

struct ImuData {
    uint64_t const timestamp_ns;

    double angular_velocity[3];
    double linear_acceleration[3];
};

bool operator<(ImuData const& x, ImuData const& y) { return x.timestamp_ns < y.timestamp_ns; }

// NOTE(Jack): Technically which database we use DOES NOT matter to the calibration process (it is an implementation
// detail only)! However instead of starting with a pure virtual interface base class to define the interface, we will
// start with a concrete implementation and if we need generalization later we will refactor.
class CalibrationDatabase {
   public:
    CalibrationDatabase(std::string const& db_path, bool const create, bool const read_only = false);

    CalibrationDatabase(CalibrationDatabase const& other) = delete;

    CalibrationDatabase(CalibrationDatabase&& other) noexcept = delete;

    CalibrationDatabase& operator=(CalibrationDatabase const& other) = delete;

    CalibrationDatabase& operator=(CalibrationDatabase&& other) noexcept = delete;

    ~CalibrationDatabase();

    [[nodiscard]] bool AddImuData(std::string const& sensor_name, ImuData const& data);

    std::optional<std::set<ImuData>> GetImuData(std::string const& sensor_name) const;

    [[nodiscard]] bool AddImage(std::string const& sensor_name, uint64_t const timestamp_ns, cv::Mat const& image);

   private:
    sqlite3* db_;
};

}  // namespace reprojection::database