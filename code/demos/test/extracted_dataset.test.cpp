#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "calibration/initialize_focal_length.hpp"
#include "demos/image_source.hpp"
#include "eigen_utilities/grid.hpp"
#include "geometry/lie.hpp"
#include "optimization/nonlinear_refinement.hpp"
#include "pnp/dlt.hpp"
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

    std::vector<Vector6d> dlt_poses;
    std::vector<Vector6d> nl_poses;
    for (const auto& [timestamp, target] : data) {
        std::vector<double> const fs{calibration::InitializeFocalLength(
            target, calibration::InitializationMethod::ParabolaLine, Vector2d{256, 256})};

        Array6d const ds_intrinsics{156.6221, 156.598, 254.974, 256.97158, -0.180193, 0.5910415};
        auto const ds_camera{projection_functions::DoubleSphereCamera(ds_intrinsics)};
        MatrixX3d const rays{ds_camera.Unproject(target.bundle.pixels)};

        // ERROR(Jack): We are not accounting for the fact of valid field of views!
        Array4d const pinhole_intrinsics{1, 1, 0, 0};
        auto const pinhole_camera{projection_functions::PinholeCamera(pinhole_intrinsics)};
        MatrixX2d const pixels{pinhole_camera.Project(rays)};

        Bundle const to_opt{pixels, target.bundle.points};

        Isometry3d const tf{pnp::Dlt22(to_opt)};
        dlt_poses.push_back(geometry::Log(tf));

        auto const [tf_star, _1, reprojection_error]{optimization::CameraNonlinearRefinement(
            {{{to_opt.pixels, to_opt.points}, tf}}, CameraModel::Pinhole, pinhole_intrinsics)};
        nl_poses.push_back(geometry::Log(tf_star[0]));
    }

    saveVector6dListToFile(nl_poses, "nl_poses.txt");
    saveVector6dListToFile(dlt_poses, "dlt_poses.txt");
}

Eigen::Isometry3d pnpEigen(const Eigen::MatrixX3d& points3D,  // Nx3
                           const Eigen::MatrixX2d& pixels2D,  // Nx2
                           const Eigen::Matrix3d& K,          // 3x3 intrinsics
                           bool useExtrinsicGuess = false) {
    // --- 1. Convert Eigen → cv::Mat ---
    cv::Mat objectPoints(points3D.rows(), 3, CV_64F);
    cv::Mat imagePoints(pixels2D.rows(), 2, CV_64F);

    for (int i = 0; i < points3D.rows(); ++i) {
        objectPoints.at<double>(i, 0) = points3D(i, 0);
        objectPoints.at<double>(i, 1) = points3D(i, 1);
        objectPoints.at<double>(i, 2) = points3D(i, 2);

        imagePoints.at<double>(i, 0) = pixels2D(i, 0);
        imagePoints.at<double>(i, 1) = pixels2D(i, 1);
    }

    cv::Mat cameraMatrix(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) cameraMatrix.at<double>(r, c) = K(r, c);

    cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);  // modify if you have distortion

    // --- 2. Run solvePnP ---
    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, useExtrinsicGuess,
                           cv::SOLVEPNP_ITERATIVE);

    if (!ok) throw std::runtime_error("solvePnP failed!");

    // --- 3. Convert rvec → rotation matrix ---
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);

    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) R(r, c) = R_cv.at<double>(r, c);
        t(r) = tvec.at<double>(r);
    }

    // --- 4. Build Eigen Isometry (4×4 pose) ---
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = R;
    T.translation() = t;

    return T;
}

TEST(DemosExtractedDataset, TestOpenCv) {
    std::map<double, ExtractedTarget> const data{demos::LoadExtractedTargets(
        "/data/cvg.cit.tum.de_visual-inertial-dataset/dataset-calib-imu4_512_16_extracted/imgs")};

    std::vector<Vector6d> opencv_poses;
    for (const auto& [timestamp, target] : data) {
        std::vector<double> const fs{calibration::InitializeFocalLength(
            target, calibration::InitializationMethod::ParabolaLine, Vector2d{256, 256})};

        Array6d const ds_intrinsics{156.6221, 156.598, 254.974, 256.97158, -0.180193, 0.5910415};
        auto const ds_camera{projection_functions::DoubleSphereCamera(ds_intrinsics)};
        MatrixX3d const rays{ds_camera.Unproject(target.bundle.pixels)};

        // ERROR(Jack): We are not accounting for the fact of valid field of views!
        Array4d const pinhole_intrinsics{1, 1, 0, 0};
        auto const pinhole_camera{projection_functions::PinholeCamera(pinhole_intrinsics)};
        MatrixX2d const pixels{pinhole_camera.Project(rays)};

        Bundle const to_opt{pixels, target.bundle.points};

        opencv_poses.push_back(geometry::Log(pnpEigen(to_opt.points, to_opt.pixels, Matrix3d::Identity())));
        opencv_poses.back().bottomRows(3) *= 10;
    }

    saveVector6dListToFile(opencv_poses, "opencv_poses.txt");
}