#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

/**
 * \ingroup projection_classes
 */
struct Pinhole {
    static int constexpr Size{4};

    // NOTE(Jack): The following notation is used for all projection functions, but we put it here in the pinhole class
    // at is the most basic projection function and starting point for all others.
    //
    // There are three reference frames/point types in the projection process. These are,
    //
    //      P_co: {x, y, z} - A 3D point in the camera optical ("*_co") frame
    //      p_cam: {x_cam, y_cam} - A 2D point in the "ideal" or "normalized" camera frame.
    //      pixel: {u, v} - A 2D pixel in image space coordinates.
    //
    // Note that in most literature and implementations p_cam is actually treated as a homogeneous point,
    //
    //      p_cam = {x, y, z} / z --> {x/z, y/z, 1}
    //
    // This allows for the 3x3 K matrix to be applied directly as a matrix multiplication,
    //
    //      pixel = K * {x/z, y/z, 1}^T
    //
    // We represent it as a non-homogeneous point in the code, but achieve all the same goals as the more
    // common/familiar homogeneous representation.
    //
    // For the pinhole camera model we see in the implementation that applying "ideal projection" to P_co produces p_cam
    // and that applying the camera matrix K to p_cam gives us a pixel. Understanding this basic pattern is crucial for
    // understanding how other camera models are implemented as some variation of this basic process.
    template <typename T>
    static Eigen::Array<T, 2, 1> Project(Eigen::Array<T, Size, 1> const& intrinsics,
                                         Eigen::Array<T, 3, 1> const& P_co) {
        T const& x{P_co[0]};
        T const& y{P_co[1]};
        T const& z{P_co[2]};
        // Put into ideal/normalized/projected camera coordinate frame
        T const x_cam{x / z};
        T const y_cam{y / z};

        T const& fx{intrinsics[0]};
        T const& fy{intrinsics[1]};
        T const& cx{intrinsics[2]};
        T const& cy{intrinsics[3]};
        // Put into image pixel space
        T const u{(fx * x_cam) + cx};
        T const v{(fy * y_cam) + cy};
        Eigen::Array<T, 2, 1> const pixel{u, v};

        return pixel;
    }

    static Array3d Unproject(Eigen::Array<double, Size, 1> const& intrinsics, Eigen::Array<double, 2, 1> const& pixel);
};

}  // namespace reprojection::projection_functions