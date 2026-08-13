import logging
import textwrap
from pathlib import Path

from business_logic.geometry import Se3ToMat
from business_logic.toml_conversions import toml_to_intrinsic_array
from dashboard.tools.data_loading import refresh_database_list
from database.data_formatting import (
    parse_workflows,
    process_workflow,
)
from database.sql_table_loading import load_calibration_database
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

        db = load_calibration_database(db_path)
        workflows = parse_workflows(db)

        output = []

        for workflow in workflows:
            # TODO(Jack): Do we need a better way to handle multiple workflows in one file? Yes!
            workflow_output = f"[workflow{workflow.id}]\n" f"type = '{workflow.type}'\n"
            output.append(workflow_output)

            workflow_data = process_workflow(db, workflow)

            if workflow.type in ("cam", "cam_imu"):
                cam_result = build_intrinsic_toml(workflow, workflow_data)
                if len(cam_result) != 0:
                    output.append(cam_result)

            if workflow.type == "cam_imu":
                extrinsic_result = build_extrinsic_toml(workflow, workflow_data)
                if extrinsic_result is not None:
                    output.append(extrinsic_result)

        if len(output) == 0:
            log.info(f"No calibration data exported for {db_name}")
            continue

        output_name = db_name.removesuffix(".db3") + ".toml"
        output_path = Path(workspace_dir) / output_name
        with open(output_path, "w") as f:
            f.write("\n".join(output))

        log.info(
            "Saving calibration toml:\n%s",
            textwrap.indent(f"Name: {output_name}\nPath: {output_path}", "  "),
        )


def build_intrinsic_toml(workflow, workflow_data):
    camera = workflow.assets.get("camera")
    camera_info = workflow_data.get("camera_info")
    camera_intrinsics = workflow_data.get("intrinsics")

    if camera is None or camera_info is None or camera_intrinsics is None:
        return ""

    if camera_info.empty or camera_intrinsics.empty:
        return ""

    bundle_adjustment_step_id = workflow.steps.get("bundle_adjustment")
    if bundle_adjustment_step_id is None:
        return ""

    sensor_name = camera["name"]
    asset_id = camera["id"]

    log.info(f"Processing intrinsic {sensor_name}")

    camera_intrinsic_row = camera_intrinsics.loc[(bundle_adjustment_step_id, asset_id)]

    intrinsics_str = camera_intrinsic_row["data"]
    camera_model = CameraModel(camera_info["camera_model"])
    intrinsics_arr = toml_to_intrinsic_array(intrinsics_str, camera_model)

    camera_index = camera["index"]

    return (
        f"[workflow{workflow.id}.cam{camera_index}]\n"
        f"sensor_id = '{sensor_name}'\n"
        f"camera_model = '{camera_info['camera_model']}'\n"
        f"intrinsics = {intrinsics_arr}\n"
        f"resolution = [{int(camera_info['height'])}, {int(camera_info['width'])}]\n"
    )


def build_extrinsic_toml(workflow, workflow_data):
    extrinsics = workflow_data.get("extrinsics")
    if extrinsics is None or extrinsics.empty:
        return None

    camera = workflow.assets.get("camera")
    imu = workflow.assets.get("imu")

    if camera is None or imu is None:
        return None

    data = extrinsics.iloc[0]

    log.info(f"Processing extrinsic {camera['name']} -> {imu['name']}")

    se3_a_b = data[["rx", "ry", "rz", "x", "y", "z"]].to_numpy().squeeze()
    tf_a_b = Se3ToMat(se3_a_b)

    def format_toml_matrix(matrix, precision: int = 12) -> str:
        rows = []
        for row in matrix:
            values = ", ".join(f"{float(value):.{precision}g}" for value in row)
            rows.append(f"  [{values}]")
        return "[\n" + ",\n".join(rows) + "\n]"

    return (
        f"[workflow{workflow.id}.extrinsic0]\n"
        f"frame_a = '{camera['name']}'\n"
        f"frame_b = '{imu['name']}'\n"
        f"tf_a_b = {format_toml_matrix(tf_a_b)}\n"
    )
