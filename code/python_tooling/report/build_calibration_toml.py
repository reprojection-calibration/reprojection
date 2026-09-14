import logging
import textwrap
from pathlib import Path

from business_logic.geometry import Se3ToMat
from business_logic.toml_conversions import toml_to_intrinsic_array
from database.data_formatting import parse_workflows, process_workflow, select_rows
from database.discovery import refresh_database_list
from database.sql_table_loading import load_calibration_database
from database.types import CameraModel

log = logging.getLogger("reprojection")


def run_toml_export(workspace_dir):
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
            # TODO(Jack): Do we need a better way to handle multiple workflows in one file?
            workflow_output = f"[workflow{workflow.id}]\n" f"type = '{workflow.type}'\n"
            output.append(workflow_output)

            workflow_data = process_workflow(db, workflow)

            cam_result = build_intrinsic_toml(workflow, workflow_data)
            if cam_result:
                output.append(cam_result)
            extrinsic_result = build_extrinsic_toml(workflow, workflow_data)
            if extrinsic_result:
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
    camera_info = workflow_data.get("camera_info")
    camera_intrinsics = workflow_data.get("intrinsics")

    if camera_info is None or camera_intrinsics is None:
        return ""

    if camera_info.empty or camera_intrinsics.empty:
        return ""

    output = []
    for camera in workflow.assets_of_type("camera"):
        sensor_name = camera["name"]
        asset_id = camera["id"]
        bundle_adjustment_step_ids = workflow.step_ids(
            "bundle_adjustment", asset_id=asset_id
        )

        if not bundle_adjustment_step_ids:
            continue

        info_rows = select_rows(camera_info, asset_id=asset_id)
        intrinsic_rows = select_rows(
            camera_intrinsics, asset_id=asset_id, step_ids=bundle_adjustment_step_ids
        )
        if info_rows.empty or intrinsic_rows.empty:
            continue
        camera_info_row = info_rows.iloc[0]

        log.info(f"Processing intrinsic {sensor_name}")
        camera_intrinsic_row = intrinsic_rows.iloc[0]

        intrinsics_str = camera_intrinsic_row["data"]
        camera_model = CameraModel(camera_intrinsic_row["camera_model"])
        intrinsics_arr = toml_to_intrinsic_array(intrinsics_str, camera_model)

        camera_index = camera["index"]
        output.append(
            f"[workflow{workflow.id}.cam{camera_index}]\n"
            f"sensor_id = '{sensor_name}'\n"
            f"camera_model = '{camera_intrinsic_row['camera_model']}'\n"
            f"intrinsics = {intrinsics_arr}\n"
            f"resolution = [{int(camera_info_row['height'])}, {int(camera_info_row['width'])}]\n"
        )

    return "\n".join(output)


def format_toml_matrix(matrix, precision=12):
    rows = [
        "  [" + ", ".join(f"{float(value):.{precision}g}" for value in row) + "]"
        for row in matrix
    ]
    return "[\n" + ",\n".join(rows) + "\n]"


def build_extrinsic_toml(workflow, workflow_data):
    extrinsics = workflow_data.get("extrinsics")
    if extrinsics is None or extrinsics.empty:
        return None

    output = []
    for index, (_, row) in enumerate(
        extrinsics.sort_values(["step_id", "asset_a_id", "asset_b_id"]).iterrows()
    ):
        frame_a = workflow.assets[row["asset_a_id"]]
        frame_b = workflow.assets[row["asset_b_id"]]
        log.info(f"Processing extrinsic {frame_b['name']} -> {frame_a['name']}")
        tf_a_b = Se3ToMat(row[["rx", "ry", "rz", "x", "y", "z"]].to_numpy())
        output.append(
            f"[workflow{workflow.id}.extrinsic{index}]\n"
            f"step_id = {int(row['step_id'])}\n"
            f"frame_a = '{frame_a['name']}'\n"
            f"frame_b = '{frame_b['name']}'\n"
            f"tf_a_b = {format_toml_matrix(tf_a_b)}\n"
        )
    return "\n".join(output)
