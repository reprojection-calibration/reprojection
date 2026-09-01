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
    asset_group_signature: str
    assets: dict[int, dict]
    steps: dict[int, dict]

    def assets_of_type(self, asset_type):
        return [asset for asset in self.assets.values() if asset["type"] == asset_type]

    def step_ids(self, step_type=None, asset_id=None):
        matching_step_ids = []
        for step_id, step in self.steps.items():
            if step_type is not None and step["type"] != step_type:
                continue
            if asset_id is not None and asset_id not in self.step_asset_groups_ids(
                step_id
            ):
                continue

            matching_step_ids.append(step_id)

        return matching_step_ids

    # TODO(Jack): This is hardcoding that our asset group signature format will not changes and will continue to habe
    # the same semantics over time. Not a deal breaker but something we need to keep in mind - its coding a lot into a
    # single little string.
    def step_asset_groups_ids(self, step_id):
        signature = self.steps[step_id]["asset_group_signature"]

        return {int(value) for value in signature.split("|") if value}


def parse_workflows(db):
    workflows = []
    for _, workflow_row in db["workflows"].iterrows():
        workflow_id = int(workflow_row["id"])

        asset_ids = db["workflow_assets"].loc[
            db["workflow_assets"]["workflow_id"] == workflow_id,
            "asset_id",
        ]
        assets = (
            db["assets"]
            .loc[db["assets"]["id"].isin(asset_ids)]
            .set_index("id", drop=False)
            .to_dict("index")
        )

        step_rows = db["workflow_steps"].loc[
            db["workflow_steps"]["workflow_id"] == workflow_id
        ]
        steps = {
            int(row["step_id"]): {
                "type": row["type"],
                "asset_group_signature": row["asset_group_signature"],
            }
            for _, row in step_rows.iterrows()
        }

        workflows.append(
            Workflow(
                id=workflow_id,
                type=workflow_row["type"],
                asset_group_signature=workflow_row["asset_group_signature"],
                assets=assets,
                steps=steps,
            )
        )

    return workflows


# Extract all the rows from a specific table for a single asset that are from a step in the given workflow.
def all_step_rows(table, workflow, asset_id):
    if table is None or table.empty:
        return pd.DataFrame()

    mask = table.index.get_level_values("step_id").isin(workflow.steps)
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
    asset_ids = set(workflow.assets)
    for table_name in (
        "camera_info",
        "target_info",
        "camera_poses",
        "extracted_targets",
        "images_timestamps",
        "imu_data",
        "imu_errors",
        "intrinsics",
        "reprojection_errors",
    ):
        table = db.get(table_name)
        if table is None or table.empty:
            continue
        mask = table.index.get_level_values("step_id").isin(workflow.steps)
        mask &= table.index.get_level_values("asset_id").isin(asset_ids)
        rows = table.loc[mask]
        if not rows.empty:
            workflow_data[table_name] = rows

    # NOTE(Jack): Extrinsics are unique because they belong to a pair of assets. Here we use the logic that there
    # can only be one extrinsic for any workflow sensor pair. If this is bulletproof I am not sure.
    extrinsic_step_ids = workflow.step_ids("extrinsic_optimization")
    extrinsics = db.get("extrinsics")

    if extrinsic_step_ids and extrinsics is not None:
        mask = extrinsics.index.get_level_values("step_id").isin(extrinsic_step_ids)
        workflow_data["extrinsics"] = extrinsics.loc[mask]

    return workflow_data


def to_legacy_data(workflow, workflow_data):
    data = {}

    # step_id -> step type
    step_types = {step_id: step["type"] for step_id, step in workflow.steps.items()}

    for camera in workflow.assets_of_type("camera"):
        camera_name = camera["name"]

        data[camera_name] = {
            "type": SensorType.Camera,
            "measurements": {
                "images": {},
            },
        }

        # Images
        table = asset_rows(workflow_data.get("images_timestamps"), camera["id"])
        if table is not None and not table.empty:
            for _, row in table.iterrows():
                timestamp_ns = int(row["timestamp_ns"])
                data[camera_name]["measurements"]["images"][timestamp_ns] = None

        # Camera info
        camera_info = asset_rows(workflow_data.get("camera_info"), camera["id"])
        if camera_info is not None and not camera_info.empty:
            camera_info = camera_info.iloc[0]
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
            target_info = target_info.iloc[0]
            data[camera_name]["target_info"] = {
                "target_type": TargetType(target_info["target_type"]),
                "height": target_info["height"],
                "width": target_info["width"],
                "unit_dimension": target_info["unit_dimension"],
                "asymmetric": bool(target_info["asymmetric"]),
            }

        # Extracted targets
        table = asset_rows(workflow_data.get("extracted_targets"), camera["id"])
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
        table = asset_rows(workflow_data.get("camera_poses"), camera["id"])
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
        table = asset_rows(workflow_data.get("reprojection_errors"), camera["id"])
        if table is not None and not table.empty:
            reprojection_errors = {}

            for (step_id, _), row in table.iterrows():
                step_type = step_types[step_id]
                timestamp_ns = int(row["timestamp_ns"])

                reprojection_errors.setdefault(step_type, {})[timestamp_ns] = row[
                    "data"
                ]

            data[camera_name]["reprojection_error"] = reprojection_errors

    for imu in workflow.assets_of_type("imu"):
        imu_name = imu["name"]

        data[imu_name] = {
            "type": SensorType.Imu,
            "measurements": {},
        }

        # IMU measurements
        table = asset_rows(workflow_data.get("imu_data"), imu["id"])
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
        table = asset_rows(workflow_data.get("imu_errors"), imu["id"])
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


def asset_rows(table, asset_id):
    if table is None or table.empty:
        return pd.DataFrame()

    mask = table.index.get_level_values("asset_id") == asset_id

    return table.loc[mask]
