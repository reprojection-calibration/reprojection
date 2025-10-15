#include "target_extractors.hpp"

#include <iostream>

extern "C" {
#include "generated_apriltag_code/tagCustom36h11.h"
}

#include "utilities.hpp"

namespace reprojection_calibration::feature_extraction {

CheckerboardExtractor::CheckerboardExtractor(cv::Size const& pattern_size, const double unit_dimension)
    : TargetExtractor(pattern_size, unit_dimension) {
    point_indices_ = GenerateGridIndices(pattern_size_.height, pattern_size_.width);
    points_ = Eigen::MatrixX3d{point_indices_.rows(), 3};
    points_.leftCols(2) = unit_dimension_ * point_indices_.cast<double>();
    points_.col(2).setZero();  // Flat on calibration board, z=0.
}

std::optional<FeatureFrame> CheckerboardExtractor::Extract(cv::Mat const& image) const {
    std::vector<cv::Point2f> corners;
    bool const pattern_found{cv::findChessboardCorners(
        image, pattern_size_, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK)};

    if (not pattern_found) {
        return std::nullopt;
    }

    cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

    return FeatureFrame{ToEigen(corners), points_, point_indices_};
}

CircleGridExtractor::CircleGridExtractor(cv::Size const& pattern_size, const double unit_dimension,
                                         bool const asymmetric)
    : TargetExtractor(pattern_size, unit_dimension), asymmetric_{asymmetric} {
    if (asymmetric_) {
        // NOTE(Jack): We reverse the order of the width and height here for the asymmetric case! Why that is... you
        // tell me boss...
        point_indices_ = GenerateGridIndices(pattern_size.width, pattern_size.height, true);
    } else {
        point_indices_ = GenerateGridIndices(pattern_size_.height, pattern_size_.width);
    }

    points_ = Eigen::MatrixX3d{point_indices_.rows(), 3};
    points_.leftCols(2) = unit_dimension_ * point_indices_.cast<double>();
    points_.col(2).setZero();
}

std::optional<FeatureFrame> CircleGridExtractor::Extract(cv::Mat const& image) const {
    // cv::CALIB_CB_CLUSTERING - "uses a special algorithm for grid detection. It is more robust to perspective
    // distortions but much more sensitive to background clutter." - if I do not use this then I think I need to do
    // some tuning about what acceptable sizes and spacing are for the circle grid. For now this will do.
    // TODO(Jack): This is not so clean here because we will have to repeat all options (ex.
    // cv::CALIB_CB_CLUSTERING) even though those will probably be the same for both cases. Keep your eyes peeled
    // for associated problems!
    int const extraction_options{asymmetric_ ? cv::CALIB_CB_CLUSTERING | cv::CALIB_CB_ASYMMETRIC_GRID
                                             : cv::CALIB_CB_CLUSTERING | cv::CALIB_CB_SYMMETRIC_GRID};

    // NOTE(Jack): Something which violates the principle of least surprise is how OpenCV deals with the dimension of
    // asymmetric circle grids. There are two things which are curious to me; #1 that we have to switch the height and
    // width order for the asymmetric case and #2 that we need to divide one of the dimension by two!
    cv::Size pattern_size{pattern_size_};
    if (asymmetric_) {
        pattern_size = cv::Size{pattern_size_.height / 2, pattern_size_.width};
    }

    std::vector<cv::Point2f> corners;
    bool const pattern_found{cv::findCirclesGrid(image, pattern_size, corners, extraction_options)};

    if (not pattern_found) {
        return std::nullopt;
    };

    return FeatureFrame{ToEigen(corners), points_, point_indices_};
}

// NOTE(Jack): Use of the tagCustom36h11 and all settings are hardcoded here! This means no on can select another
// family. Find a way to make this configurable if possible, but it will likely require recompilation.
AprilGrid3Extractor::AprilGrid3Extractor(cv::Size const& pattern_size, const double unit_dimension)
    : TargetExtractor(pattern_size, unit_dimension),
      tag_family_{AprilTagFamily{tagCustom36h11_create(), tagCustom36h11_destroy}},
      tag_detector_{AprilTagDetector{tag_family_, {2.0, 0.0, 1, false, false}}} {
    point_indices_ = GenerateGridIndices(2 * pattern_size_.height, 2 * pattern_size_.width);
    points_ = CornerPositions(point_indices_, unit_dimension);
}

std::optional<FeatureFrame> AprilGrid3Extractor::Extract(cv::Mat const& image) const {
    std::vector<AprilTagDetection> const raw_detections{tag_detector_.Detect(image)};
    if (std::size(raw_detections) == 0) {
        return std::nullopt;
    }

    Eigen::MatrixX2d corners{4 * std::size(raw_detections), 2};
    for (size_t i{0}; i < std::size(raw_detections); ++i) {
        // WARN(Jack): The homography can launch the corners outside the bound of the image, this is currently not
        // handled, and how that shows up in our code is not yet clear (2.10.2025).
        Eigen::Matrix<double, 4, 2> const extraction_corners{
            EstimateExtractionCorners(raw_detections[i].H, std::sqrt(tag_family_.tag_family->nbits))};
        Eigen::Matrix<double, 4, 2> const refined_extraction_corners{RefineCorners(image, extraction_corners)};

        corners.block<4, 2>(4 * i, 0) = refined_extraction_corners;
    }

    Eigen::ArrayXi const mask{AprilGrid3Extractor::VisibleGeometry(pattern_size_, raw_detections)};

    // TODO(Jack): Make corner and point naming consistent!
    return FeatureFrame{corners, points_(mask, Eigen::all), point_indices_(mask, Eigen::all)};
}

// This function is responsible for handling the more complex indexing and grid arrangement that is inherent to an
// april board. Our goal when dealing with the april board is to produce rows and columns of points exactly like a
// chekerboard or circle grid. Because of the nature of april tags (having four points for each tag) and the
// requirement to handle different size boards and different metric size tags, this is not trivial. This function is
// the critical building block that is responsible for providing a mask which tells us, given the detection tag IDs,
// which points are visible and which are not.
Eigen::ArrayXi AprilGrid3Extractor::VisibleGeometry(cv::Size const& pattern_size,
                                                    std::vector<AprilTagDetection> const& detections) {
    std::vector<int> mask;
    for (auto const& detection : detections) {
        int const i{static_cast<int>(detection.id / pattern_size.width)};
        int const j{detection.id % pattern_size.width};

        int const corner_0{(2 * (2 * i) * pattern_size.width) + (2 * j)};
        int const corner_1{corner_0 + 1};
        int const corner_2{corner_0 + (2 * pattern_size.width)};
        int const corner_3{corner_2 + 1};

        mask.push_back(corner_0);
        mask.push_back(corner_1);
        mask.push_back(corner_2);
        mask.push_back(corner_3);
    }

    return ToEigen(mask);
}

Eigen::MatrixX3d AprilGrid3Extractor::CornerPositions(Eigen::ArrayX2i const& indices, double const unit_dimension) {
    Eigen::MatrixX3d points{indices.rows(), 3};
    for (int i{0}; i < indices.rows(); ++i) {
        // WARN(Jack): If we change the pattern (num_bits or design) then this 0.4 (4bits/10bits) will change! This
        // function is currently assuming that AprilBoard3 will be fixed forever using the custom 36h11 tag family.
        points.row(i)(0) = AlternatingSum(indices.row(i)(1), unit_dimension, 0.4 * unit_dimension);
        points.row(i)(1) = AlternatingSum(indices.row(i)(0), unit_dimension, 0.4 * unit_dimension);
    }
    points.col(2).setZero();

    return points;
}

// From the apriltag documentation (https://github.com/AprilRobotics/apriltag/blob/master/apriltag.h)
//
//      The 3x3 homography matrix describing the projection from an "ideal" tag (with corners at (-1,1), (1,1),
//      (1,-1), and (-1,-1)) to pixels in the image.
//
// Here the "corner" positions correspond to the four corners on the inside of the black ring that defines the
// "quad" of an April Tag 3. In the tags designed for use in the April Board 3, the corners that we want to extract
// and use are found on the outside of this black ring, at the intersection of the black ring and the corner
// element. This intersection is designed to provide the characteristic checkerboard like intersection which can be
// refined using the cv::cornerSubPix() function to provide nearly exact corner pixel coordinates. ADD , int const
// num_bits
Eigen::Matrix<double, 4, 2> AprilGrid3Extractor::EstimateExtractionCorners(Eigen::Matrix3d const& H,
                                                                           int const sqrt_num_bits) {
    // NOTE(Jack): These corners have been reordered from how they are listed in the april tag documentation. The
    // current ordering matches our generated targets grid row/column indexing.
    Eigen::Matrix<double, 4, 2> const canonical_corners{{-1, -1}, {1, -1}, {-1, 1}, {1, 1}};
    double const corner_offset_scale{(sqrt_num_bits / 2.0 + 2.0) / (sqrt_num_bits / 2.0 + 1.0)};

    Eigen::Matrix<double, 4, 2> extraction_corners{
        (H * (corner_offset_scale * canonical_corners).rowwise().homogeneous().transpose())
            .transpose()
            .rowwise()
            .hnormalized()};

    return extraction_corners;
}

Eigen::Matrix<double, 4, 2> AprilGrid3Extractor::RefineCorners(cv::Mat const& image,
                                                               Eigen::Matrix<double, 4, 2> const& extraction_corners) {
    // NOTE(Jack): Eigen is column major by default, but opencv is row major (like the rest of the world...) so we
    // need to specifically specify Eigen::RowMajor here in order for the cv::Mat view to make sense.
    Eigen::Matrix<float, 4, 2, Eigen::RowMajor> refined_extraction_corners{extraction_corners.cast<float>()};
    cv::Mat cv_view_extraction_corners(refined_extraction_corners.rows(), refined_extraction_corners.cols(), CV_32FC1,
                                       refined_extraction_corners.data());  // cv::cornerSubPix() requires float type

    cv::cornerSubPix(image, cv_view_extraction_corners, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

    return refined_extraction_corners.cast<double>();
}

}  // namespace reprojection_calibration::feature_extraction