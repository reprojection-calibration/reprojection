#pragma once

#include <memory>
#include <vector>

#include "projection_functions/camera_model.hpp"
#include "spline/se3_spline.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

struct MvgFrame {
    Isometry3d pose;
    MatrixX2d pixels;
    MatrixX3d points;
};

// MVG = "multiple view geometry"
class MvgGenerator {
   public:
    explicit MvgGenerator(std::unique_ptr<projection_functions::Camera> const camera, bool const flat = true);

    // Honestly we often almost always want the full batch, and then iterate over it, is there really a point to not
    // only support the batch api? Otherwise, we have a lot of looping logic copy and pasted throughout the testing
    // code. Furthermore,- if we control/only allow creation of the batch then we reduce the risk that someone requests
    // and invalid time (i.e. outside 0 to 1). Asking the user "how many frames do you want might?" even just be more
    // intuitive over all.
    std::vector<MvgFrame> GenerateBatchFrames(int const num_frames) const;

    // This an override which serves up types more amenable for use by the nonlinear optimization. At this point we
    // have no consistent logical strategy therefore we need these conversion methods. Hopefully we will figure out how
    // to handle the data values soon and remove such methods.
    std::tuple<std::vector<MatrixX2d>, std::vector<MatrixX3d>, std::vector<Isometry3d>> GenerateBatch(
        int const num_frames) const;

    static Eigen::MatrixX2d Project(MatrixX3d const& points_w,
                                    std::unique_ptr<projection_functions::Camera> const& camera,
                                    Isometry3d const& tf_co_w);

   private:
    // Input is fractional time of trajectory from [0,1)
    MvgFrame Generate(double const t) const;

    static Eigen::MatrixX3d BuildTargetPoints(bool const flat);

    std::unique_ptr<projection_functions::Camera> camera_;
    spline::Se3Spline se3_spline_;
    Eigen::MatrixX3d points_;
};

}  // namespace reprojection::testing_mocks