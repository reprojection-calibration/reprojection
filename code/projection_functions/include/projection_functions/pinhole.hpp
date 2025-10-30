#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

// TODO(Jack): Enforce projection function interface
struct Pinhole {
    static int constexpr Size{4};

    // TODO(Jack): Move this documentation to a future common location for all projection functions
    // Applying ideal projection to "P_co" gives us "p_cam". Applying the calibration values of K to "p_cam" gives us
    // "pixel".
    //
    //      P_co: {x, y, z}
    //      p_cam: {x_cam, y_cam}
    //      pixel: {u, v}

    // NOTE(Jack): For the sake of verbosity/explicitness this function includes some lines operations which could be
    // combined but are instead done explicitly for emphasis of the relevant coordinate frames.
    template <typename T>
    static Eigen::Array<T, 2, 1> Project(Eigen::Array<T, Size, 1> const& intrinsics,
                                         Eigen::Array<T, 3, 1> const& P_co) {
        // Put into ideal/normalized/projected camera coordinate frame
        T const& x{P_co[0]};
        T const& y{P_co[1]};
        T const& z{P_co[2]};

        T const x_cam{x / z};
        T const y_cam{y / z};

        // Put into pixelized image coordinate frame
        T const& fx{intrinsics[0]};
        T const& fy{intrinsics[1]};
        T const& cx{intrinsics[2]};
        T const& cy{intrinsics[3]};

        T const u{(fx * x_cam) + cx};
        T const v{(fy * y_cam) + cy};

        Eigen::Array<T, 2, 1> const pixel{u, v};

        return pixel;
    }

    static Array3d Unproject(Eigen::Array<double, Size, 1> const& intrinsics, Eigen::Array<double, 2, 1> const& pixel);
};

}  // namespace reprojection::projection_functions