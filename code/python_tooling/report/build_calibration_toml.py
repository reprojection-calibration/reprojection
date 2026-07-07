import logging
import textwrap
from pathlib import Path

from business_logic.geometry import Se3ToMat
from business_logic.toml_conversions import toml_to_intrinsic_array
from dashboard.tools.data_loading import refresh_database_list
from database.sql_table_loading import (
    load_camera_info_table,
    load_camera_intrinsics_table,
    load_extrinsics_table,
)
from database.types import CameraModel

log = logging.getLogger("reprojection")


# TODO(Jack): The logic in this function is super hacky. We need to clean this up big time, in particular the flow
# control regarding when we "continue" and when not. For example what if there is no camera calibration in a database
# but there is an extrinsic calibration there is no reason we cannot export the extrinsic calibration. However in the
# current version this is not allowed. It is just an ugly piece of code and needs some love.
def run_toml_export(workspace_dir):
    # TODO(Jack): Using refresh_database_list here means that it is not really specific to the dashboard itself, maybe we
    # should move it to another location.
    db_list, _ = refresh_database_list(workspace_dir)

    for entry in db_list:
        db_name = entry["label"]
        db_path = entry["value"]
        log.info(
            "Generating calibration toml for:\n%s",
            textwrap.indent(f"Name: {db_name}\nPath: {db_path}", "  "),
        )

        camera_info = load_camera_info_table(db_path)
        camera_intrinsics = load_camera_intrinsics_table(db_path)
        if camera_info is None or camera_intrinsics is None:
            log.info(
                "Skipping intrinsic toml export - missing data:\n%s",
                textwrap.indent(
                    f"camera_info: {'N/A' if camera_info is None else 'loaded'}\ncamera_intrinsics: {'N/A' if camera_intrinsics is None else 'loaded'}",
                    "  ",
                ),
            )
            continue

        cam_result = build_intrinsic_toml(camera_info, camera_intrinsics)
        if len(cam_result) == 0:
            log.info(f"No camera intrinsics exported for {db_name}")
            continue

        extrinsics = load_extrinsics_table(db_path)
        extrinsic_result = None
        if extrinsics is not None:
            extrinsic_result = build_extrinsic_toml(extrinsics)

        output_name = db_name.removesuffix(".db3") + ".calib.toml"
        output_path = Path(workspace_dir) / output_name
        with open(output_path, "w") as f:
            f.write(cam_result)

            if extrinsic_result is not None:
                f.write("\n")
                f.write(extrinsic_result)

        log.info(
            "Saving calibration toml:\n%s",
            textwrap.indent(f"Name: {output_name}\nPath: {output_path}", "  "),
        )


def build_intrinsic_toml(camera_info, camera_intrinsics):
    # NOTE(Jack): For now we only care about the "polished/final" intrinsics from the camera only nonlinear refinement
    # step. This might change one day with the stereo or IMU calibration but that is future music :)
    refined_intrinsics = camera_intrinsics[
        camera_intrinsics["step_name"] == "bundle_adjustment"
    ]

    output = []
    for i, (_, sensor) in enumerate(camera_info.iterrows()):
        sensor_name = sensor["sensor_name"]
        log.info(f"Processing intrinsic {sensor_name}")

        camera_intrinsic_row = refined_intrinsics[
            refined_intrinsics["sensor_name"] == sensor_name
        ]

        if camera_intrinsic_row.empty:
            log.warning(f"No intrinsics for sensor {sensor_name}")
            continue

        intrinsics_str = camera_intrinsic_row.iloc[0]["intrinsics"]
        camera_model = CameraModel(sensor["camera_model"])
        intrinsics_arr = toml_to_intrinsic_array(intrinsics_str, camera_model)

        toml_text = (
            f"[cam{i}]\n"
            f"sensor_id = '{sensor_name}'\n"
            f"# https://github.com/reprojection-calibration/reprojection#camera-models\n"
            f"camera_model = '{sensor['camera_model']}'\n"
            f"intrinsics = {intrinsics_arr}\n"
            f"resolution = [{int(sensor['height'])}, {int(sensor['width'])}]\n"
        )

        output.append(toml_text)

    return "\n".join(output)


def build_extrinsic_toml(extrinsics):
    # NOTE(Jack): Just like for the camera intrinsics we only want to give the user the final optimized version, not the
    # intermediate rough initialization.
    optimized_extrinsics = extrinsics[
        extrinsics["step_name"] == ("extrinsic_optimization")
    ]

    output = []
    for i, (_, data) in enumerate(optimized_extrinsics.iterrows()):
        entity_id = data["entity_id"]
        log.info(f"Processing extrinsic {entity_id}")

        se3_a_b = data[["rx", "ry", "rz", "x", "y", "z"]].to_numpy().squeeze()
        tf_a_b = Se3ToMat(se3_a_b)

        def format_toml_matrix(matrix, precision: int = 12) -> str:
            rows = []
            for row in matrix:
                values = ", ".join(f"{float(value):.{precision}g}" for value in row)
                rows.append(f"  [{values}]")
            return "[\n" + ",\n".join(rows) + "\n]"

        toml_text = (
            f"[extrinsic{i}]\n"
            f"frame_a = '{data['frame_a']}'\n"
            f"frame_b = '{data['frame_b']}'\n"
            f"tf_a_b = {format_toml_matrix(tf_a_b)}\n"
        )

        output.append(toml_text)

    return "\n".join(output)
