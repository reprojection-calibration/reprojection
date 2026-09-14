from dash import html

from dashboard.tools.selection import table_rows


def extract_labeled_metadata(data, parent_keys=None):
    if parent_keys is None:
        parent_keys = []

    statistics = []

    for key, value in data.items():
        current_path = parent_keys + [str(key)]

        if isinstance(value, dict):
            statistics.extend(extract_labeled_metadata(value, current_path))
        else:
            statistics.append((current_path, value))

    return statistics


# TODO(Jack): Test?!
def build_sensor_statistics_html(sensor_metadata):
    stats = extract_labeled_metadata(sensor_metadata)
    stats = sorted(stats, key=lambda x: x[0][0])

    stat_cards = []
    for key, value in stats:
        is_ok = value != 0

        stat_cards.append(
            html.Div(
                [
                    html.Div(
                        [
                            html.Div(
                                style={
                                    "width": "10px",
                                    "height": "10px",
                                    "borderRadius": "50%",
                                    "backgroundColor": "green" if is_ok else "red",
                                    "marginRight": "6px",
                                }
                            ),
                            html.Div(
                                key[0],
                                style={
                                    "fontSize": "13px",
                                    "fontWeight": "500",
                                },
                            ),
                        ],
                        style={
                            "display": "flex",
                            "alignItems": "center",
                            "marginBottom": "6px",
                        },
                    ),
                    html.Div(
                        str(value),
                        style={
                            "fontSize": "18px",
                            "fontWeight": "bold",
                        },
                    ),
                    html.Div(
                        key[-1] if len(key) > 1 else "",
                        style={
                            "fontSize": "12px",
                            "color": "#666",
                        },
                    ),
                ],
                style={
                    "minWidth": "200px",
                    "padding": "10px",
                    "backgroundColor": "white",
                    "border": "1px solid #ddd",
                    "borderRadius": "6px",
                    "boxShadow": "0px 1px 2px rgba(0,0,0,0.05)",
                },
            )
        )

    return stat_cards


def step_selector_options(asset_id, metadata):
    if asset_id is None or not metadata:
        return [], None

    artifact_steps = {
        count["step_id"]
        for count in metadata["counts"]
        if count["asset_id"] == asset_id
        and count["table"] in ("camera_poses", "reprojection_errors")
        and count["count"] > 0
    }
    options = [
        {"label": f"{step['type']} ({step['step_id']})", "value": step["step_id"]}
        for step in metadata["steps"]
        if step["step_id"] in artifact_steps
    ]
    options.sort(key=lambda option: (option["label"], option["value"]))
    return options, options[0]["value"] if options else None


def build_sensor_metadata_layout(asset_id, metadata, workflow_data):
    if asset_id is None or not metadata:
        return []

    # Table and step remain visible instead of inferring steps from nested keys.
    statistics = {}
    for count in metadata["counts"]:
        if count["asset_id"] == asset_id:
            statistics.setdefault(count["table"], {})[str(count["step_id"])] = count[
                "count"
            ]
    for row in table_rows(workflow_data, "camera_info", asset_id=asset_id):
        statistics[f"camera_info (step {row['step_id']})"] = {
            key: value
            for key, value in row.items()
            if key not in ("step_id", "asset_id")
        }
    # Target descriptions belong to workflow target assets, not the selected camera.
    assets = {asset["id"]: asset for asset in metadata["assets"]}
    for row in table_rows(workflow_data, "target_info"):
        target = assets[row["asset_id"]]
        label = f"Target {target['name']} ({target['id']}, step {row['step_id']})"
        statistics[label] = {
            key: value
            for key, value in row.items()
            if key not in ("step_id", "asset_id")
        }
    return build_sensor_statistics_html(statistics)
