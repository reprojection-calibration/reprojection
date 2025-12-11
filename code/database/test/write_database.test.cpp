#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "database/calibration_database.hpp"
#include "database/sensor_data_interface.hpp"
#include "sqlite3_helpers.hpp"

namespace reprojection::text_files {

template <typename EigenType>
EigenType LoadEigenArray(std::filesystem::path const& file) {
    std::ifstream in(file);
    if (!in) {
        throw std::runtime_error("Failed to open " + file.string());
    }

    using T = EigenType::Scalar;
    std::vector<T> buffer;
    buffer.reserve(1024);

    T value;
    while (in >> value) {
        buffer.push_back(value);
    }

    size_t constexpr cols{EigenType::ColsAtCompileTime};
    if (std::size(buffer) % cols != 0) {
        throw std::runtime_error("Column mismatch in " + file.string());
    }
    size_t const rows{std::size(buffer) / cols};

    EigenType out(rows, cols);
    for (size_t r{0}; r < rows; ++r) {
        for (size_t c{0}; c < cols; ++c) {
            out(r, c) = buffer[r * cols + c];
        }
    }

    return out;
}

std::map<double, ExtractedTarget> LoadExtractedTargets(std::filesystem::path const& root) {
    std::vector<double> timestamps;
    {
        std::ifstream tsfile(root / "timestamps.txt");
        if (not tsfile) {
            throw std::runtime_error("Missing timestamps.txt in " + root.string());
        }

        double t;
        while (tsfile >> t) {
            timestamps.push_back(t);
        }
    }

    namespace fs = std::filesystem;
    std::vector<fs::path> frames_directories;
    for (auto const& entry : fs::directory_iterator(root)) {
        if (entry.is_directory()) {
            std::string const name{entry.path().filename().string()};
            if (std::all_of(std::cbegin(name), std::cend(name), isdigit)) {
                frames_directories.push_back(entry.path());
            }
        }
    }

    std::sort(std::begin(frames_directories), std::end(frames_directories));

    if (std::size(frames_directories) != std::size(timestamps)) {
        throw std::runtime_error("Mismatch between frame folders and timestamps.");
    }

    std::map<double, ExtractedTarget> result;
    for (size_t i{0}; i < std::size(frames_directories); i++) {
        fs::path const& dir_i{frames_directories[i]};

        ArrayX2i const ids{LoadEigenArray<ArrayX2i>(dir_i / "ids.txt")};
        MatrixX2d const pixels{LoadEigenArray<MatrixX2d>(dir_i / "pixels.txt")};
        MatrixX3d const points{LoadEigenArray<MatrixX3d>(dir_i / "points.txt")};

        if (ids.rows() != pixels.rows() || ids.rows() != points.rows()) {
            throw std::runtime_error("Row count mismatch in " + dir_i.string());
        }

        result.emplace(timestamps[i], ExtractedTarget{{pixels, points}, ids});
    }

    return result;
}

std::vector<database::ImuData> loadImuFile(const std::string& filename) {
    std::vector<database::ImuData> data;
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');
        // Skip empty lines or comments
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        database::ImuData imu;

        // Read timestamp + 6 doubles
        if (!(iss >> imu.timestamp_ns >> imu.angular_velocity[0] >> imu.angular_velocity[1] >>
              imu.angular_velocity[2] >> imu.linear_acceleration[0] >> imu.linear_acceleration[1] >>
              imu.linear_acceleration[2])) {
            std::cerr << "Warning: malformed line, skipping:\n" << line << "\n";
            continue;
        }

        data.push_back(imu);
    }

    return data;
}

}  // namespace reprojection::text_files

using namespace reprojection;

TEST(Xxxx, Yyyyy) {
    std::string const record_path{"dataset-calib-imu4_512_16.db3"};
    auto db{std::make_shared<database::CalibrationDatabase>(record_path, true, false)};

    std::map<double, ExtractedTarget> const cam0_data{text_files::LoadExtractedTargets("/data/cam0")};
    (void)database::Sqlite3Tools::Execute("BEGIN TRANSACTION;", db->db);
    for (auto const& [time, target] : cam0_data) {
        uint64_t const timestamp_ns_i{static_cast<uint64_t>(time * 1e9)};
        EXPECT_TRUE(AddExtractedTargetData("/cam0/image_raw", {timestamp_ns_i, target}, db));
    }
    (void)database::Sqlite3Tools::Execute("COMMIT TRANSACTION;", db->db);

    std::map<double, ExtractedTarget> const cam1_data{text_files::LoadExtractedTargets("/data/cam1")};
    (void)database::Sqlite3Tools::Execute("BEGIN TRANSACTION;", db->db);
    for (auto const& [time, target] : cam1_data) {
        uint64_t const timestamp_ns_i{static_cast<uint64_t>(time * 1e9)};
        EXPECT_TRUE(AddExtractedTargetData("/cam1/image_raw", {timestamp_ns_i, target}, db));
    }
    (void)database::Sqlite3Tools::Execute("COMMIT TRANSACTION;", db->db);

    std::vector<database::ImuData> const imu_data{text_files::loadImuFile(
        "/data/cvg.cit.tum.de_visual-inertial-dataset/dataset-calib-imu4_512_16/mav0/imu0/data.csv")};

    (void)database::Sqlite3Tools::Execute("BEGIN TRANSACTION;", db->db);
    for (auto const& imu_i : imu_data) {
        EXPECT_TRUE(database::AddImuData("/imu0", imu_i, db));
    }
    (void)database::Sqlite3Tools::Execute("COMMIT TRANSACTION;", db->db);
}