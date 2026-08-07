from dataclasses import dataclass

# TODO(Jack): Does it not make more sense to store the dictionary time keys as strings to prevent any problems with
#  dash/json serialization?


@dataclass
class Workflow:
    id: int
    type: str
    signature: str
    assets: list[dict]
    steps: list[dict]


def parse_workflows(db):
    workflows = []
    for _, workflow_row in db["workflows"].iterrows():
        workflow_id = int(workflow_row["id"])

        asset_ids = db["workflow_assets"].loc[
            db["workflow_assets"]["workflow_id"] == workflow_id, "asset_id"
        ]
        assets = db["assets"].loc[db["assets"]["id"].isin(asset_ids)].to_dict("records")

        steps = (
            db["workflow_steps"]
            .loc[db["workflow_steps"]["workflow_id"] == workflow_id]
            .drop(columns="workflow_id")
            .to_dict("records")
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
