#include "target_generators.hpp"

#include "utilities.hpp"

namespace reprojection_calibration::feature_extraction {

// TODO(Jack): Consider using cv:Size instead of rows and cols.
cv::Mat GenerateCheckerboard(cv::Size const& pattern_size, int const square_size_pixels) {
    // TODO(Jack): Which concept should the user be familiar with, the "internal rows/cols" or the checkerboards
    // rows/cols themselves
    int const rows{pattern_size.height + 1};
    int const cols{pattern_size.width + 1};

    int const white_space_border{2 * square_size_pixels};  // White space around checkerboard aids extraction
    int const height{(square_size_pixels * rows) + white_space_border};
    int const width{(square_size_pixels * cols) + white_space_border};
    cv::Mat checkerboard{255 * cv::Mat::ones(height, width, CV_8UC1)};  // Start with white image

    Eigen::ArrayX2i const grid{GenerateGridIndices(rows, cols)};
    for (Eigen::Index i{0}; i < grid.rows(); ++i) {
        Eigen::Array2i const indices{grid.row(i)};

        // This condition gives us an asymmetric grid - like that of  checkerboard/chessboard
        if (indices.sum() % 2 == 0) {
            cv::Point const top_left_corner{(square_size_pixels * indices(1)) + (white_space_border / 2),
                                            (square_size_pixels * indices(0)) + (white_space_border / 2)};
            cv::Point const bottom_right_corner{top_left_corner.x + square_size_pixels,
                                                top_left_corner.y + square_size_pixels};
            cv::rectangle(checkerboard, top_left_corner, bottom_right_corner, (0), -1);
        }
    }

    return checkerboard;
}

// NOTE(Jack): The unit dimension for the circle is its radius!
// NOTE(Jack): The circles themselves are the features, not the intersection between the circles, therefore the indexing
// logic will be different than by the checkerboard - i.e the checkboard always works in the rows+1 or cols+1 space,
// whereas the circle grid will works on rows and cols directly.
cv::Mat GenerateCircleGrid(cv::Size const& pattern_size, int const circle_radius_pixels,
                           int const circle_spacing_pixels, bool const asymmetric) {
    int const circumference{2 * circle_radius_pixels};
    int const white_space_border{2 * circumference};  // White space around checkerboard aids extraction
    // WARN(Jack): Is this "pattern_size.* -3" logic make sense? Are we at risk of a throwing a negative number here? We
    // might need to add some conditions to prevent that.
    int const height{(circumference * pattern_size.height) + (circle_spacing_pixels * (pattern_size.height - 3)) +
                     white_space_border};
    int const width{(circumference * pattern_size.width) + (circle_spacing_pixels * (pattern_size.width - 3)) +
                    white_space_border};
    cv::Mat circlgrid{255 * cv::Mat::ones(height, width, CV_8UC1)};

    Eigen::ArrayX2i const grid{asymmetric ? GenerateGridIndices(pattern_size.height, pattern_size.width, true)
                                          : GenerateGridIndices(pattern_size.height, pattern_size.width, false)};

    for (Eigen::Index i{0}; i < grid.rows(); ++i) {
        Eigen::Array2i const indices{grid.row(i)};

        // THERE IS SOMETHING WRONG! THE WHITE SPACE BORDER IS NOT ONE FULL CIRCLE CIRCUMFERENCE
        cv::Point const center{circle_radius_pixels + (white_space_border / 2) +
                                   (circle_spacing_pixels * (indices(1) - 1)) + (circumference * indices(1)),
                               circle_radius_pixels + (white_space_border / 2) +
                                   (circle_spacing_pixels * (indices(0) - 1)) + (circumference * indices(0))};
        cv::circle(circlgrid, center, circle_radius_pixels, (0), -1);
    }

    return circlgrid;
}

cv::Mat AprilBoard3Generation::GenerateBoard(int const num_bits, uint64_t const tag_family[], int const bit_size_pixels,
                                             cv::Size const& pattern_size) {
    int const april_tag_size_pixels{
        (8 * bit_size_pixels) +
        (static_cast<int>(std::sqrt(num_bits)) * bit_size_pixels)};  // Fixed border width plus dynamic data area size

    cv::Mat april_board{cv::Mat::zeros(pattern_size.height * april_tag_size_pixels,
                                       pattern_size.width * april_tag_size_pixels, CV_8UC1)};

    Eigen::ArrayX2i const tag_layout{GenerateGridIndices(pattern_size.height, pattern_size.width)};
    for (Eigen::Index i{0}; i < tag_layout.rows(); ++i) {
        Eigen::Array2i const indices{tag_layout.row(i)};

        // Generate the tag
        uint64_t const tag_code_i{tag_family[(indices(0) * pattern_size.width) + indices(1)]};
        cv::Mat const april_tag_i{GenerateTag(num_bits, tag_code_i, bit_size_pixels)};

        // Place the tag on the board
        cv::Point const top_left_corner{(april_tag_size_pixels * indices(1)), (april_tag_size_pixels * indices(0))};
        cv::Point const bottom_right_corner{top_left_corner.x + april_tag_size_pixels,
                                            top_left_corner.y + april_tag_size_pixels};
        cv::Rect const roi{cv::Rect(top_left_corner, bottom_right_corner)};
        april_tag_i.copyTo(april_board(roi));
    }

    return april_board;
}

cv::Mat AprilBoard3Generation::GenerateTag(int const num_bits, uint64_t const tag_code, int const bit_size_pixels) {
    Eigen::MatrixXi const code_matrix{GenerateCodeMatrix(num_bits, tag_code)};

    return GenerateTag(bit_size_pixels, code_matrix);
}

cv::Mat AprilBoard3Generation::GenerateTag(int const bit_size_pixels, Eigen::MatrixXi const& code_matrix) {
    int const border_thickness_pixels{
        4 * bit_size_pixels};  // Three mainly white rings and one black ring. This is an intrinsic property
                               // of the tags in our proposed AprilBoard3 design.
    int const num_bits{static_cast<int>(code_matrix.rows())};  // Could also use .cols().

    int const tag_size_pixels{2 * border_thickness_pixels + (num_bits * bit_size_pixels)};
    cv::Mat april_tag{255 * cv::Mat::ones(tag_size_pixels, tag_size_pixels, CV_8UC1)};  // Tags are square

    // Hacky way to draw the surrounding black edge rectangle - we cannot use cv::rectangle directly because we need to
    // have constant thickness. cv::rectangle will round corners of partially filled rectangles.
    {
        // Fill in the entire center black
        cv::Point const top_left_corner{2 * bit_size_pixels, 2 * bit_size_pixels};
        cv::Point const bottom_right_corner{(6 + num_bits) * bit_size_pixels - 1, (6 + num_bits) * bit_size_pixels - 1};
        cv::rectangle(april_tag, top_left_corner, bottom_right_corner, (0), -1);
    }
    {
        // Fill back in the center white - leaving a black rim one bit thick.
        cv::Point const top_left_corner{3 * bit_size_pixels, 3 * bit_size_pixels};
        cv::Point const bottom_right_corner{(5 + num_bits) * bit_size_pixels, (5 + num_bits) * bit_size_pixels};
        cv::rectangle(april_tag, top_left_corner, bottom_right_corner, (255), -1);
    }

    {
        // Put in the corner sharpening elements.
        cv::Mat const corner_element{cv::Mat::zeros(2 * bit_size_pixels, 2 * bit_size_pixels, CV_8UC1)};

        // Put the corner sharpening element we just created into all four corners of the tag. We rotate the april tag
        // itself to save ourselves the annoying math of calculating the rectangle corner point locations.
        cv::Rect const roi{cv::Rect(cv::Point{0, 0}, cv::Point{2 * bit_size_pixels, 2 * bit_size_pixels})};
        for (int i{0}; i < 4; ++i) {
            corner_element.copyTo(april_tag(roi));
            cv::rotate(april_tag, april_tag, cv::ROTATE_90_CLOCKWISE);
        }
    }

    // Fill in all the bits of the data region
    // TODO(Jack): This logic is practically exactly the same as in the checkerboard generation... is there a practical
    // way to DRY ourselves here?
    Eigen::ArrayX2i const grid{GenerateGridIndices(num_bits, num_bits)};
    for (Eigen::Index i{0}; i < grid.rows(); ++i) {
        Eigen::Array2i const indices{grid.row(i)};

        if (code_matrix(indices(0), indices(1))) {
            cv::Point const top_left_corner{(bit_size_pixels * indices(1)) + border_thickness_pixels,
                                            (bit_size_pixels * indices(0)) + border_thickness_pixels};
            cv::Point const bottom_right_corner{top_left_corner.x + bit_size_pixels,
                                                top_left_corner.y + bit_size_pixels};
            cv::rectangle(april_tag, top_left_corner, bottom_right_corner, (0), -1);
        }
    }

    return april_tag;
}

// To understand this method you need to read the renderToArray() method in ImageLayout.java
// (https://github.com/AprilRobotics/apriltag-generation/blob/master/src/april/tag/ImageLayout.java)
// We simplify the implementation a little bit here because we assume that AprilBoard3 tag structure will always be the
// same, therefore we only duplicate the actual data code area generation part here. A complete implementation of april
// tag generation is not found here! Please see the original AprilRobotics repository for that.
Eigen::MatrixXi AprilBoard3Generation::GenerateCodeMatrix(int const num_bits, uint64_t tag_code) {
    int const sqrt_num_bits{static_cast<int>(std::sqrt(num_bits))};  // Only allow square data encoding areas

    // TODO(Jack): Is there a hard reason that we need to do this by generating each quadrant and then rotating 90
    // degrees? If not, and there is time and energy, find an implementation that just lets us iterate over a normal set
    // of indices like one would expect.
    Eigen::MatrixXi code_matrix(sqrt_num_bits, sqrt_num_bits);
    for (int k{0}; k < 4; ++k) {
        for (int i{0}; i <= sqrt_num_bits / 2; ++i) {
            for (int j{i}; j < sqrt_num_bits - 1 - i; ++j) {
                uint64_t const bit_sign{(tag_code & (static_cast<uint64_t>(1) << (num_bits - 1)))};
                code_matrix(j, i) = not bit_sign;  // I switched i and j from what I thought they should be, and then it
                                                   // started working... the entire repo needs a check of consistency
                                                   // for the ordering of its indices.

                tag_code = tag_code << 1;
            }
        }
        code_matrix = Rotate90(code_matrix, true);
    }

    // Set center pixel if there is one (i.e. odd number of bits) - this pixel will not be set in the 90 degree quadrant
    // rotations above -_-.
    if (sqrt_num_bits % 2 != 0) {
        uint64_t const bit_sign{(tag_code & (static_cast<uint64_t>(1) << (num_bits - 1)))};  // COPY AND PASTE
        code_matrix(sqrt_num_bits / 2, sqrt_num_bits / 2) = not bit_sign;
    }

    // WARN(Jack): Why I need this transpose I am not 100% sure, but compared to the official implementation we look
    // mirrored without this step. This entire method is slightly hacked and this is simply that coming to the surface.
    return code_matrix.transpose();
}

Eigen::MatrixXi AprilBoard3Generation::Rotate90(Eigen::MatrixXi const& matrix, bool const clockwise) {
    Eigen::MatrixXi const matrix_star{matrix.transpose()};

    return clockwise ? matrix_star.rowwise().reverse().eval() : matrix_star.colwise().reverse().eval();
}

}  // namespace reprojection_calibration::feature_extraction