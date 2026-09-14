from dataclasses import dataclass

import pandas as pd


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
            if asset_id is not None and asset_id not in self.step_asset_ids(step_id):
                continue

            matching_step_ids.append(step_id)

        return matching_step_ids

    # TODO(Jack): This is hardcoding that our asset group signature format will not changes and will continue to have
    # the same semantics over time. Not a deal breaker but something we need to keep in mind - its coding a lot into a
    # single little string.
    def step_asset_ids(self, step_id):
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


def select_rows(table, *, asset_id=None, step_ids=None):
    """Select SQL rows without changing their columns or identities."""
    if table is None:
        return pd.DataFrame()

    rows = table
    if asset_id is not None:
        rows = rows.loc[rows["asset_id"] == asset_id]
    if step_ids is not None:
        rows = rows.loc[rows["step_id"].isin(step_ids)]
    return rows


def process_workflow(db, workflow):
    """Keep artifacts belonging to the workflow's steps and assets."""
    tables = {}
    for name, table in db.items():
        if "step_id" not in table or name == "workflow_steps":
            continue
        rows = select_rows(table, step_ids=workflow.steps)
        if "asset_id" in rows:
            rows = rows.loc[rows["asset_id"].isin(workflow.assets)]
        elif "asset_a_id" in rows and "asset_b_id" in rows:
            rows = rows.loc[
                rows["asset_a_id"].isin(workflow.assets)
                & rows["asset_b_id"].isin(workflow.assets)
            ]
        if not rows.empty:
            tables[name] = rows
    return tables
