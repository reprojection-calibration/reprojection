from business_logic.toml_conversions import toml_to_intrinsic_array
from dashboard.tools.data_loading import refresh_database_list
from database.sql_table_loading import (
    load_camera_info_table,
    load_camera_intrinsics_table,
)
from database.types import CameraModel


def run_toml_export(workspace_dir):
    # TODO(Jack): Using refresh_database_list here means that it is not really specific to the dashboard itself, maybe we
    # should move it to another location.
    db_list, _ = refresh_database_list(workspace_dir)

    for entry in db_list:
        db_name = entry["label"]
        db_path = entry["value"]
        print(f"Generating camera intrinsic toml for database {db_name}")

        camera_info = load_camera_info_table(db_path)
        camera_intrinsics = load_camera_intrinsics_table(db_path)

        result = build_camera_toml(camera_info, camera_intrinsics)

        if len(result) == 0:
            print("\tNo camera intrinsics exported for", db_name, "\n")
            continue

        output_name = db_name.removesuffix(".db3") + ".toml"
        output_path = workspace_dir / output_name
        with open(output_path, "w") as f:
            f.write(result)

        print(f"\tAssembled camera intrinsic toml and saving to {output_path}")


def build_camera_toml(camera_info, camera_intrinsics):
    # NOTE(Jack): For now we only care about the "polished/final" intrinsics from the camera only nonlinear refinement
    # step. This might change one day with the stereo or IMU calibration but that is future music :)
    refined_intrinsics = camera_intrinsics[
        camera_intrinsics["step_name"] == "camera_nonlinear_refinement"
    ]

    output = []
    for i, sensor in camera_info.iterrows():
        sensor_name = sensor["sensor_name"]
        print(f"\tProcessing sensor {sensor_name}")

        camera_intrinsic_row = refined_intrinsics[
            refined_intrinsics["sensor_name"] == sensor_name
        ]

        if camera_intrinsic_row.empty:
            print(f"\t\t No intrinsics for sensor {sensor_name}")
            continue

        intrinsics_str = camera_intrinsic_row.iloc[0]["intrinsics"]
        camera_model = CameraModel(sensor["camera_model"])
        intrinsics_arr = toml_to_intrinsic_array(intrinsics_str, camera_model)

        toml_text = (
            f"[cam{i}]\n"
            f'sensor_id = "{sensor_name}"\n'
            f"# https://github.com/reprojection-calibration/reprojection#camera-models\n"
            f'camera_model = "{sensor["camera_model"]}"\n'
            f"intrinsics = {intrinsics_arr}\n"
            f'resolution = [{int(sensor["height"])}, {int(sensor["width"])}]'
            "\n"
        )

        output.append(toml_text)

    return "\n".join(output)
