from dataclasses import dataclass

import pandas as pd

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


def process_workflow(db, workflow):
    workflow_data = {}

    # TODO(Jack): This a hard requirement then that every workflow has a camera and a target. Is there a way to avoid this?
    camera_id = workflow.assets["camera"]["id"]
    target_id = workflow.assets["target"]["id"]

    single_tables = {
        "camera_info": (workflow.steps["camera_info"], camera_id),
        "target_info": (workflow.steps["target_info"], target_id),
    }
    for table_name, (step_id, asset_id) in single_tables.items():
        workflow_data[table_name] = row_or_empty(
            db[table_name],
            step_id,
            asset_id,
        )

    # NOTE(Jack): Hardcodes that we only load camera asset tables. Makes sense for now.
    for table_name in (
        "camera_poses",
        "extracted_targets",
        "images_timestamps",
        "reprojection_errors",
    ):
        workflow_data[table_name] = all_step_rows(
            db[table_name],
            workflow,
            camera_id,
        )
        print(workflow_data)
