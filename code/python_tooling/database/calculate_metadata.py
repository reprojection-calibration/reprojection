def workflow_metadata(workflow, tables):
    """Count artifact rows by their SQL identities, including both extrinsic frames."""
    counts = []
    for name, table in tables.items():
        asset_columns = [
            column
            for column in ("asset_id", "asset_a_id", "asset_b_id")
            if column in table
        ]
        for asset_column in asset_columns:
            for (step_id, asset_id), count in (
                table.groupby(["step_id", asset_column]).size().items()
            ):
                counts.append(
                    {
                        "table": name,
                        "step_id": int(step_id),
                        "asset_id": int(asset_id),
                        "count": int(count),
                    }
                )
    return {
        "assets": list(workflow.assets.values()),
        "steps": [
            dict(
                step_id=step_id,
                asset_ids=sorted(workflow.step_asset_ids(step_id)),
                **step
            )
            for step_id, step in workflow.steps.items()
        ],
        "counts": counts,
    }
