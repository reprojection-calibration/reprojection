#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "calibration/initialize_focal_length.hpp"
#include "demos/image_source.hpp"
#include "geometry/lie.hpp"
#include "pnp/pnp.hpp"
#include "projection_functions/camera_model.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::demos {

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

}  // namespace reprojection::demos

using namespace reprojection;

bool saveVector6dListToFile(const std::vector<Vector6d>& vec_list, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file: " << filename << std::endl;
        return false;
    }

    for (const auto& v : vec_list) {
        // Write the six components separated by spaces
        file << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " " << v(4) << " " << v(5) << "\n";
    }

    file.close();
    return true;
}

TEST(DemosExtractedDataset, TestXXX) {
    std::map<double, ExtractedTarget> const data{demos::LoadExtractedTargets(
        "/data/cvg.cit.tum.de_visual-inertial-dataset/dataset-calib-imu4_512_16_extracted/imgs")};

    std::vector<Vector6d> poses;
    for (const auto& [timestamp, target] : data) {
        std::vector<double> const fs{calibration::InitializeFocalLength(
            target, calibration::InitializationMethod::ParabolaLine, Vector2d{256, 256})};

        double lowest_error{1e10};
        Isometry3d best_pose;
        ;
        for (auto const f : fs) {
            Array6d const ds_intrinsics{f / 2, f / 2, 256, 256, 0, 0.5};
            auto const ds_camera{projection_functions::DoubleSphereCamera(ds_intrinsics)};
            MatrixX3d const rays{ds_camera.Unproject(target.bundle.pixels)};

            // ERROR(Jack): We are not accounting for the fact of valid field of views!
            Array4d const pinhole_intrinsics{1, 1, 0, 0};
            auto const pinhole_camera{projection_functions::PinholeCamera(pinhole_intrinsics)};
            MatrixX2d const pixels{pinhole_camera.Project(rays)};

            pnp::PnpResult const pnp_result{pnp::Pnp({pixels, target.bundle.points})};

            EXPECT_TRUE(std::holds_alternative<pnp::PnpOutput>(pnp_result));
            auto const result{std::get<pnp::PnpOutput>(pnp_result)};
            if (result.reprojection_error < lowest_error) {
                lowest_error = result.reprojection_error;
                best_pose = result.pose;
            }
        }
        poses.push_back(geometry::Log(best_pose.inverse()));
    }

    saveVector6dListToFile(poses, "tum_dataset_poses.txt");

}