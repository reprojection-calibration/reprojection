#pragma once

#include <filesystem>

namespace reprojection::database {

namespace fs = std::filesystem;

struct AssetId {
    size_t value;
};

enum class AssetType { Camera, Imu, Target };

std::string ToString( AssetType const data) {
    if (data == AssetType::Camera) {
        return "camera";
    } else if (data == AssetType::Imu) {
        return "imu";
    }else if (data == AssetType::Target) {
        return "target";
    }else {
        throw std::runtime_error("Unknown AssetType - Library implementation error.");
    }
}

class CalibrationDatabase {
   public:
    // TODO(Jack): Should we make this private and instead use a factory?
    CalibrationDatabase(fs::path const& db_path, bool const create, bool const read_only = false);

    AssetId GetOrCreateAsset(AssetType const type,  size_t const index, std::string_view name);

};

}  // namespace reprojection::database