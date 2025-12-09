#pragma once

#include <sqlite3.h>

#include <string>

namespace reprojection::database {

// NOTE(Jack): Technically which database we use DOES NOT matter to the calibration process (it is an implementation
// detail only)! However instead of starting with a pure virtual interface base class to define the interface, we will
// start with a concrete implementation and if we need generalization later we will refactor.
struct CalibrationDatabase {
    CalibrationDatabase(std::string const& db_path, bool const create, bool const read_only = false);

    CalibrationDatabase(CalibrationDatabase const& other) = delete;

    CalibrationDatabase(CalibrationDatabase&& other) noexcept = delete;

    CalibrationDatabase& operator=(CalibrationDatabase const& other) = delete;

    CalibrationDatabase& operator=(CalibrationDatabase&& other) noexcept = delete;

    ~CalibrationDatabase();

    sqlite3* db;
};

}  // namespace reprojection::database