from business_logic.toml_conversions import toml_to_intrinsic_array
from database.types import CameraModel


def build_camera_tomls(camera_info, camera_intrinsics):
    # NOTE(Jack): For now we only care about the "polished/final" intrinsics from the camera only nonlinear refinement
    # step. This might change one day with the stereo or IMU calibration but that is future music :)
    refined_intrinsics = camera_intrinsics[
        camera_intrinsics["step_name"] == "camera_nonlinear_refinement"
    ]

    output = []
    for i, sensor in camera_info.iterrows():
        sensor_name = sensor["sensor_name"]
        camera_intrinsic_row = refined_intrinsics[
            refined_intrinsics["sensor_name"] == sensor_name
        ]

        if camera_intrinsic_row.empty:
            continue

        intrinsics_str = camera_intrinsic_row.iloc[0]["intrinsics"]
        camera_model = CameraModel(sensor["camera_model"])
        intrinsics_arr = toml_to_intrinsic_array(intrinsics_str, camera_model)

        toml_text = (
            f"[cam{i}]\n"
            f'sensor_id = "{sensor_name}"\n'
            f'camera_model = "{sensor["camera_model"]}"\n'
            f"intrinsics = {intrinsics_arr}\n"
            f'resolution = [{int(sensor["height"])}, {int(sensor["height"])}]'
        )

        output.append(toml_text)

    return "\n".join(output)
