#pragma once

#include <filesystem>
#include <memory>
#include <variant>

// Forward declaration so we can hide the real db headers from the consuming applications.
namespace reprojection::database {

struct CalibrationDatabase;

}

namespace reprojection::application {

namespace fs = std::filesystem;
using DbPtr = std::shared_ptr<database::CalibrationDatabase>;

// TODO NAMING!
struct DbErrorMsg {
    std::string msg;
};

// TODO(Jack): We probably want to tell the user that the database was opened and/or if it was created new or we are
//  using an old one etc.
std::variant<DbPtr, DbErrorMsg> Open(fs::path const& workspace, fs::path const& data_source);

}  // namespace reprojection::application