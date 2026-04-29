
#include "pnp/pnp.hpp"

#include "database/calibration_database.hpp"
#include "database/database_read.hpp"
#include "eigen_utilities/grid.hpp"
#include "geometry/lie.hpp"
#include "projection_functions/initialize_camera.hpp"

using namespace reprojection;

std::optional<std::pair<FrameState, double>> EstimatePoseViaPinholePnP(std::unique_ptr<projection_functions::Camera> const& camera,
                                                                       Bundle const& bundle,
                                                                       ImageBounds const& bounds) {
    // Unproject to rays (pseudo 3D - no depth information) using the camera model provided by the user.
    auto const [rays, mask_unproject]{camera->Unproject(bundle.pixels)};

    // Project the rays using a unit ideal pinhole camera to get undistorted/linearized pixels
    auto const pinhole_camera{projection_functions::PinholeCamera({1, 1, 0, 0}, {-1, 1, -1, 1})};
    auto const [pixels, mask_project]{pinhole_camera.Project(rays)};

    // Combine the masks and make a new bundle from only the valid reprojected points.
    ArrayXb const mask{mask_unproject * mask_project};
    ArrayXi const valid_indices{eigen_utilities::MaskToRowId(mask)};
    Bundle const linearized_bundle{pixels(valid_indices, Eigen::all), bundle.points(valid_indices, Eigen::all)};

    auto const result{pnp::Pnp(linearized_bundle, bounds)};
    if (std::holds_alternative<pnp::PoseWithCost>(result)) {
        auto const [tf_co_w, cost]{std::get<pnp::PoseWithCost>(result)};
        return std::pair<FrameState, double>{geometry::Log(tf_co_w), cost};
    } else {
        return std::nullopt;  // LCOV_EXCL_LINE
    }
}

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/sunshiny_day3.db3"};
    auto db{database::OpenCalibrationDatabase(record_path, false, false)};

    CameraMeasurements const targets{database::GetExtractedTargetData(db, "/camera/image_raw")};

    // "target of interest"
    ExtractedTarget const target{targets.at(11544751989663)};

    ImageBounds const bounds{0, 1280, 0, 720};

    auto const camera{projection_functions::InitializeCamera(CameraModel::DoubleSphere,
                                                             Array6d{731.980, 731.980, 640, 360, 0, 0.5}, bounds)};

    auto const result{EstimatePoseViaPinholePnP(camera, target.bundle, bounds)};

    return EXIT_SUCCESS;
}
