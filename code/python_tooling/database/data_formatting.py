from dataclasses import dataclass


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


def process_workflow(db, workflow):
    db["camera_info"] = db["camera_info"].set_index(["step_id", "asset_id"])
    row = db["camera_info"].loc[(workflow.steps["camera_info"], workflow.assets["camera"]["id"])]

    print(row)
