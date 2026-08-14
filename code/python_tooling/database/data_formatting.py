from dataclasses import dataclass

import pandas as pd

from business_logic.geometry import InvertSe3
from database.types import SensorType, TargetType

# TODO(Jack): Does it not make more sense to store the dictionary time keys as strings to prevent any problems with
#  dash/json serialization?


@dataclass
class Workflow:
    id: int
    type: str
    signature: str
    assets: dict[str, dict]
    steps: dict[str, int]


def parse_workflows(db):
    workflows = []
    for _, workflow_row in db["workflows"].iterrows():
        workflow_id = int(workflow_row["id"])

        # NOTE(Jack): By storing the assets indexed in a dict by their asset type (i.e. camera/target/imu) we are
        # hardcoding the fact that there can only be one of each asset type in any workflow. This will not scale to
        # multisensor setups but for now it gets the job done.
        asset_ids = db["workflow_assets"].loc[
            db["workflow_assets"]["workflow_id"] == workflow_id,
            "asset_id",
        ]
        assets = (
            db["assets"]
            .loc[db["assets"]["id"].isin(asset_ids)]
            .set_index("type")
            .to_dict("index")
        )

        steps = (
            db["workflow_steps"]
            .loc[db["workflow_steps"]["workflow_id"] == workflow_id]
            .set_index("type")["step_id"]
            .to_dict()
        )

        workflows.append(
            Workflow(
                id=workflow_id,
                type=workflow_row["type"],
                signature=workflow_row["signature"],
                assets=assets,
                steps=steps,
            )
        )

    return workflows


# Extract all the rows from a specific table for a single asset that are from a step in the given workflow.
def all_step_rows(table, workflow, asset_id):
    if table is None or table.empty:
        return pd.DataFrame()

    mask = table.index.get_level_values("step_id").isin(workflow.steps.values())
    mask &= table.index.get_level_values("asset_id") == asset_id

    return table.loc[mask]


def row_or_empty(table, step_id, asset_id):
    if table is None or table.empty:
        return pd.DataFrame()

    try:
        row = table.loc[(step_id, asset_id)]
    except KeyError:
        return pd.DataFrame()

    return row


# Used for selecting all rows from a table just given a step id. This is useful for example when selecting the
# extrinsics which has two assets not just a single one we know ahead of time.
def step_rows(table, step_id):
    if table is None or table.empty:
        return pd.DataFrame()

    mask = table.index.get_level_values("step_id") == step_id

    return table.loc[mask]


def process_workflow(db, workflow):
    workflow_data = {}

    single_tables = {
        "camera_info": ("camera_info", "camera"),
        "target_info": ("target_info", "target"),
    }

    for table_name, (step_type, asset_type) in single_tables.items():
        step_id = workflow.steps.get(step_type)
        asset = workflow.assets.get(asset_type)

        if step_id is None or asset is None:
            continue

        workflow_data[table_name] = row_or_empty(
            db[table_name],
            step_id,
            asset["id"],
        )

    multi_tables = {
        "camera_poses": "camera",
        "extracted_targets": "camera",
        "images_timestamps": "camera",
        "imu_data": "imu",
        "imu_errors": "imu",
        "intrinsics": "camera",
        "reprojection_errors": "camera",
    }

    for table_name, asset_type in multi_tables.items():
        table = db.get(table_name)
        asset = workflow.assets.get(asset_type)

        if table is None or asset is None:
            continue

        workflow_data[table_name] = all_step_rows(
            db[table_name],
            workflow,
            asset["id"],
        )

    # NOTE(Jack): Extrinsics are unique because they belong to a pair of assets. Here we use the logic that there
    # can only be one extrinsic for any workflow sensor pair. If this is bulletproof I am not sure.
    extrinsic_step_id = workflow.steps.get("extrinsic_optimization")
    extrinsics = db.get("extrinsics")

    if extrinsic_step_id is not None and extrinsics is not None:

        print(extrinsic_step_id)
        workflow_data["extrinsics"] = step_rows(
            extrinsics,
            extrinsic_step_id,
        )

    return workflow_data


def to_legacy_data(workflow, workflow_data):
    data = {}

    camera = workflow.assets.get("camera")
    imu = workflow.assets.get("imu")

    # step_id -> step type
    step_types = {step_id: step_type for step_type, step_id in workflow.steps.items()}

    if camera is not None:
        camera_name = camera["name"]

        data[camera_name] = {
            "type": SensorType.Camera,
            "measurements": {
                "images": {},
            },
        }

        # Images
        table = workflow_data.get("images_timestamps")
        if table is not None and not table.empty:
            for _, row in table.iterrows():
                timestamp_ns = int(row["timestamp_ns"])
                data[camera_name]["measurements"]["images"][timestamp_ns] = None

        # Camera info
        camera_info = workflow_data.get("camera_info")
        if camera_info is not None and not camera_info.empty:
            data[camera_name]["camera_info"] = {
                "camera_model": camera_info["camera_model"],
                "height": camera_info["height"],
                "width": camera_info["width"],
            }

        # Target info
        #
        # NOTE: The legacy format stores target_info under the camera even
        # though the new schema correctly models the target as its own asset.
        target_info = workflow_data.get("target_info")
        if target_info is not None and not target_info.empty:
            data[camera_name]["target_info"] = {
                "target_type": TargetType(target_info["target_type"]),
                "height": target_info["height"],
                "width": target_info["width"],
                "unit_dimension": target_info["unit_dimension"],
                "asymmetric": bool(target_info["asymmetric"]),
            }

        # Extracted targets
        table = workflow_data.get("extracted_targets")
        if table is not None and not table.empty:
            targets = {}

            for _, row in table.iterrows():
                timestamp_ns = int(row["timestamp_ns"])
                target = row["data"]

                targets[timestamp_ns] = {
                    "pixels": target["pixels"],
                    "points": target["points"],
                    "indices": target["indices"],
                }

            data[camera_name]["measurements"]["targets"] = targets

        # Camera poses
        table = workflow_data.get("camera_poses")
        if table is not None and not table.empty:
            poses = {}

            for (step_id, _), row in table.iterrows():
                step_type = step_types[step_id]
                timestamp_ns = int(row["timestamp_ns"])

                pose_co_w = row.iloc[-6:].tolist()
                pose_w_co = InvertSe3(pose_co_w)

                poses.setdefault(step_type, {})[timestamp_ns] = pose_w_co

            data[camera_name]["poses"] = poses

        # Reprojection errors
        table = workflow_data.get("reprojection_errors")
        if table is not None and not table.empty:
            reprojection_errors = {}

            for (step_id, _), row in table.iterrows():
                step_type = step_types[step_id]
                timestamp_ns = int(row["timestamp_ns"])

                reprojection_errors.setdefault(step_type, {})[timestamp_ns] = row[
                    "data"
                ]

            data[camera_name]["reprojection_error"] = reprojection_errors

    if imu is not None:
        imu_name = imu["name"]

        data[imu_name] = {
            "type": SensorType.Imu,
            "measurements": {},
        }

        # IMU measurements
        table = workflow_data.get("imu_data")
        if table is not None and not table.empty:
            for _, row in table.iterrows():
                timestamp_ns = int(row["timestamp_ns"])

                data[imu_name]["measurements"][timestamp_ns] = [
                    row["omega_x"],
                    row["omega_y"],
                    row["omega_z"],
                    row["ax"],
                    row["ay"],
                    row["az"],
                ]

        # IMU errors
        table = workflow_data.get("imu_errors")
        if table is not None and not table.empty:
            imu_errors = {}

            for (step_id, _), row in table.iterrows():
                step_type = step_types[step_id]
                timestamp_ns = int(row["timestamp_ns"])

                imu_errors.setdefault(step_type, {})[timestamp_ns] = [
                    row["omega_x"],
                    row["omega_y"],
                    row["omega_z"],
                    row["ax"],
                    row["ay"],
                    row["az"],
                ]

            data[imu_name]["imu_error"] = imu_errors

    return data
