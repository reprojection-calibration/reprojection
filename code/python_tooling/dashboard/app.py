import os
import bisect
from dash import Dash, dcc, html, Input, Output, State
import plotly.graph_objects as go
from collections import defaultdict

# ------------------------------------------------------------
# Your database loaders (adjust imports if needed)
# ------------------------------------------------------------
from database.load_extracted_targets import load_all_extracted_targets
from database.load_poses import load_poses


# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------
DB_DIR = "../../test_data/"
X_RANGE = [0, 512]
Y_RANGE = [0, 512]
POSE_RANGE = [-10, 10]


# ------------------------------------------------------------
# Helpers
# ------------------------------------------------------------
def list_databases():
    if not os.path.exists(DB_DIR):
        return []
    return sorted(f for f in os.listdir(DB_DIR) if f.endswith(".db3"))


def extracted_targets_to_records(target_map, external_pose_map):
    """
    Builds a Dash-friendly structure.

    For each image timestamp:
      - stores init / opt pose if present
      - resolves closest external pose timestamp ONCE
      - stores ext_ts per pose
    """

    out = {}

    for sensor, time_map in target_map.items():
        out[sensor] = {}
        sorted_ts = sorted(time_map.keys())

        ext_pose_dict = external_pose_map.get(sensor, {})
        ext_ts = sorted(ext_pose_dict.keys())

        def closest_ext_ts(t):
            if not ext_ts:
                return None
            idx = bisect.bisect_left(ext_ts, t)
            if idx == 0:
                return ext_ts[0]
            if idx == len(ext_ts):
                return ext_ts[-1]
            before = ext_ts[idx - 1]
            after = ext_ts[idx]
            return before if abs(t - before) <= abs(after - t) else after

        for ts in sorted_ts:
            target = time_map[ts]
            pixels = target.pixels.tolist()

            record = {
                "x": [p[0] for p in pixels],
                "y": [p[1] for p in pixels],
                "ts": ts
            }

            if hasattr(target, "initial_pose"):
                record["pose_init"] = {
                    "pos": target.initial_pose,
                    "ext_ts": closest_ext_ts(ts)
                }

            if hasattr(target, "optimized_pose"):
                record["pose_opt"] = {
                    "pos": target.optimized_pose,
                    "ext_ts": closest_ext_ts(ts)
                }

            out[sensor][str(ts)] = record

        out[sensor]["_times"] = [str(ts) for ts in sorted_ts]
        out[sensor]["_external_poses"] = ext_pose_dict

    return out


# ------------------------------------------------------------
# Dash App
# ------------------------------------------------------------
app = Dash(__name__)

app.layout = html.Div(
    style={"width": "85%", "margin": "auto", "fontFamily": "sans-serif"},
    children=[
        html.H2("2D Targets + 3D Poses (with External GT)"),

        html.Div(
            style={"display": "flex", "gap": "10px", "marginBottom": "20px"},
            children=[
                dcc.Dropdown(
                    id="db-dropdown",
                    options=[{"label": db, "value": db} for db in list_databases()],
                    placeholder="Select database",
                    style={"width": "50%"}
                ),
                html.Button("Reload", id="reload-btn")
            ]
        ),

        dcc.Dropdown(
            id="sensor-dropdown",
            placeholder="Select sensor",
            style={"marginBottom": "20px"}
        ),

        html.Div(
            style={"display": "flex", "gap": "20px", "alignItems": "center"},
            children=[
                html.Button("▶ Play", id="play-btn", style={"width": "100px"}),
                dcc.Slider(
                    id="time-slider",
                    min=0, max=0, step=1, value=0,
                    tooltip={"always_visible": True}
                )
            ]
        ),

        dcc.Interval(id="play-interval", interval=50, disabled=True),
        dcc.Store(id="global-data-store"),

        dcc.Graph(id="pixel-plot"),
        dcc.Graph(id="pose-plot"),
    ]
)


# ------------------------------------------------------------
# Callbacks
# ------------------------------------------------------------
@app.callback(
    Output("db-dropdown", "options"),
    Input("reload-btn", "n_clicks")
)
def reload_db_list(_):
    return [{"label": db, "value": db} for db in list_databases()]


@app.callback(
    Output("global-data-store", "data"),
    Output("sensor-dropdown", "options"),
    Output("sensor-dropdown", "value"),
    Output("time-slider", "max"),
    Input("db-dropdown", "value")
)
def load_database(db_file):
    if not db_file:
        return None, [], None, 0

    db_path = os.path.join(DB_DIR, db_file)

    targets = load_all_extracted_targets(db_path)
    external_poses = load_poses(db_path, 'external', 'ground_truth')  # ⬅️ ground truth
    external_poses_dict = defaultdict(dict)
    for ts, sensor, *rest in external_poses:
        external_poses_dict[sensor][ts] = tuple(rest)

    dataset = extracted_targets_to_records(targets, external_poses_dict)

    sensors = list(dataset.keys())
    first_sensor = sensors[0]
    num_times = len(dataset[first_sensor]["_times"])

    return (
        dataset,
        [{"label": s, "value": s} for s in sensors],
        first_sensor,
        num_times - 1
    )


@app.callback(
    Output("play-interval", "disabled"),
    Output("play-btn", "children"),
    Input("play-btn", "n_clicks"),
    State("play-interval", "disabled"),
    prevent_initial_call=True
)
def toggle_play(_, disabled):
    return (not disabled), ("■ Stop" if disabled else "▶ Play")


# ------------------------------------------------------------
# Clientside callback
# ------------------------------------------------------------
app.clientside_callback(
    """
    function(n_intervals, slider_val, sensor, data_store, interval_disabled) {

        if (!data_store || !sensor || !data_store[sensor]) {
            return [0, window.dash_clientside.no_update, window.dash_clientside.no_update];
        }

        const times = data_store[sensor]["_times"];
        const maxIdx = times.length - 1;

        let nextVal = slider_val;
        if (!interval_disabled) {
            nextVal = (slider_val >= maxIdx) ? 0 : slider_val + 1;
        }

        const ts = times[nextVal];
        const frame = data_store[sensor][ts];
        const extPoses = data_store[sensor]["_external_poses"];

        // ---------------- 2D ----------------
        const pixelFig = {
            data: [{
                x: frame.x,
                y: frame.y,
                type: "scattergl",
                mode: "markers",
                marker: { size: 4, color: "red" }
            }],
            layout: {
                title: `Sensor: ${sensor} | Time: ${ts}`,
                xaxis: { range: [0, 512], scaleanchor: "y", scaleratio: 1 },
                yaxis: { range: [0, 512] },
                uirevision: "constant"
            }
        };

        // ---------------- 3D ----------------
        const traces = [];

        function addPose(pose, color, name, symbol) {
            traces.push({
                x: [pose.pos[0]],
                y: [pose.pos[1]],
                z: [pose.pos[2]],
                type: "scatter3d",
                mode: "markers",
                marker: { size: 6, color: color, symbol: symbol || "circle" },
                name: name
            });

            if (pose.ext_ts !== null && extPoses[pose.ext_ts]) {
                const gt = extPoses[pose.ext_ts];
                traces.push({
                    x: [gt[0]],
                    y: [gt[1]],
                    z: [gt[2]],
                    type: "scatter3d",
                    mode: "markers",
                    marker: { size: 6, color: "black", symbol: "x" },
                    name: `${name} GT`
                });
            }
        }

        // Plot init / opt if available
        if (frame.pose_init) {
            addPose(frame.pose_init, "blue", "Initial Pose");
        }
        if (frame.pose_opt) {
            addPose(frame.pose_opt, "green", "Optimized Pose");
        }

        // If no image poses, check closest external pose
        if (!frame.pose_init && !frame.pose_opt) {
            const extTsKeys = Object.keys(extPoses);
            if (extTsKeys.length > 0) {
                const tsNum = parseInt(frame.ts);
                let closestTs = extTsKeys[0];
                let minDiff = Math.abs(tsNum - parseInt(closestTs));
                for (let i = 1; i < extTsKeys.length; i++) {
                    const diff = Math.abs(tsNum - parseInt(extTsKeys[i]));
                    if (diff < minDiff) {
                        minDiff = diff;
                        closestTs = extTsKeys[i];
                    }
                }
                const gt = extPoses[closestTs];
                traces.push({
                    x: [gt[4]],
                    y: [gt[5]],
                    z: [gt[6]],
                    type: "scatter3d",
                    mode: "markers",
                    marker: { size: 6, color: "black", symbol: "x" },
                    name: "External Pose"
                });
            }
        }

        // Placeholder if no poses at all
        if (traces.length === 0) {
            traces.push({
                x: [0], y: [0], z: [0],
                type: "scatter3d",
                mode: "text",
                text: ["No pose data"],
                showlegend: false
            });
        }

        const poseFig = {
            data: traces,
            layout: {
                scene: {
                    xaxis: { range: [-10, 10] },
                    yaxis: { range: [-10, 10] },
                    zaxis: { range: [-10, 10] },
                    aspectmode: "cube"
                },
                title: `3D Poses | Sensor: ${sensor}`,
                uirevision: "constant"
            }
        };

        return [nextVal, pixelFig, poseFig];
    }
    """,
    Output("time-slider", "value"),
    Output("pixel-plot", "figure"),
    Output("pose-plot", "figure"),
    Input("play-interval", "n_intervals"),
    Input("time-slider", "value"),
    Input("sensor-dropdown", "value"),
    Input("global-data-store", "data"),
    State("play-interval", "disabled")
)


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------
if __name__ == "__main__":
    app.run(debug=True)
