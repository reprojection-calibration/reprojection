"""Queries over the SQL row records transported to the dashboard."""


def table_rows(workflow_data, table_name, *, asset_id=None, step_id=None):
    return [
        row
        for row in (workflow_data or {}).get("tables", {}).get(table_name, [])
        if (asset_id is None or row["asset_id"] == asset_id)
        and (step_id is None or row["step_id"] == step_id)
    ]


def selected_targets(workflow_data, asset_id, step_id):
    targets = table_rows(workflow_data, "extracted_targets", asset_id=asset_id)
    # A result identifies the exact extraction step that produced its observations.
    sources = {
        row["source_step_id"]
        for name in ("camera_poses", "reprojection_errors")
        for row in table_rows(workflow_data, name, asset_id=asset_id, step_id=step_id)
    }
    if sources:
        return [row for row in targets if row["step_id"] in sources]
    selected = [row for row in targets if row["step_id"] == step_id]
    if selected:
        return selected
    images = [row for row in targets if row["source_step_id"] == step_id]
    return images or targets
