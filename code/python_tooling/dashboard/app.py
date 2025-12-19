import os
from dash import Dash, dcc, html, Input, Output
import plotly.graph_objects as go

from database.load_extracted_targets import load_all_extracted_targets

# ------------------------
# Configuration
# ------------------------
DB_DIR = "../../test_data/"
TABLE_NAME = "pixels"
TIME_COL = "timestamp"
SENSOR_COL = "sensor"
X_COL = "x"
Y_COL = "y"

X_RANGE = [0, 512]
Y_RANGE = [0, 512]


# ------------------------
# Helpers
# ------------------------
def list_databases():
    if not os.path.exists(DB_DIR):
        return []
    return sorted([f for f in os.listdir(DB_DIR) if f.endswith(".db3")])


# ------------------------
# Dash App
# ------------------------
app = Dash(__name__)

app.layout = html.Div(
    style={"width": "85%", "margin": "auto"},
    children=[
        html.H2("2D Feature Viewer"),

        # Database + Reload
        html.Div(
            style={"display": "flex", "gap": "10px"},
            children=[
                dcc.Dropdown(
                    id="db-dropdown",
                    options=[{"label": db, "value": db} for db in list_databases()],
                    placeholder="Select a database",
                    style={"width": "50%"}
                ),
                html.Button("Reload Databases", id="reload-btn", n_clicks=0)
            ]
        ),

        # Sensor selector
        dcc.Dropdown(
            id="sensor-dropdown",
            placeholder="Select a sensor",
            style={"marginTop": "10px"}
        ),

        # Time slider
        dcc.Slider(
            id="time-slider",
            min=0,
            max=0,
            step=1,
            value=0,
            updatemode="drag",
            tooltip={"always_visible": True}
        ),

        dcc.Store(id="pixel-data-store"),

        dcc.Graph(id="pixel-plot")
    ]
)


# ------------------------
# Callbacks
# ------------------------

# Reload DB list
@app.callback(
    Output("db-dropdown", "options"),
    Input("reload-btn", "n_clicks")
)
def reload_database_list(_):
    return [{"label": db, "value": db} for db in list_databases()]


# Needed because the ExtractedTarget struct is not a JSON-serializable object
def extracted_targets_to_records(target_map):
    out = {}

    for sensor, time_map in target_map.items():
        out[sensor] = {}

        for timestamp, target in time_map.items():
            out[sensor][str(timestamp)] = {
                "pixels": target.pixels.tolist(),
                "points": target.points.tolist(),
                "indices": target.indices.tolist(),
            }
        out[sensor]["_times"] = sorted([str(ts) for ts in time_map.keys()])

    return out


@app.callback(
    Output("pixel-data-store", "data"),
    Input("db-dropdown", "value")
)
def load_db_to_store(db_file):
    if not DB_DIR + db_file:
        return None

    return extracted_targets_to_records(load_all_extracted_targets(DB_DIR + db_file))


# Populate sensors + slider when DB changes
@app.callback(
    Output("sensor-dropdown", "options"),
    Output("sensor-dropdown", "value"),
    Output("time-slider", "max"),
    Output("time-slider", "marks"),
    Output("time-slider", "value"),
    Input("pixel-data-store", "data")
)
def update_db_dependent_controls(extracted_targets):
    if not extracted_targets:
        return [], None, 0, {}, 0

    sensors = ["/cam0/image_raw", "/cam1/image_raw"]  # DO NOT HARDCODE
    times = extracted_targets["/cam0/image_raw"]["_times"]  # ONLY FROM ONE CAMERA!

    sensor_options = [{"label": s, "value": s} for s in sensors]
    marks = {i: str(t) for i, t in enumerate(times)}

    return sensor_options, sensors[0], len(times) - 1, marks, 0


# Update plot
@app.callback(
    Output("pixel-plot", "figure"),
    Input("pixel-data-store", "data"),
    Input("sensor-dropdown", "value"),
    Input("time-slider", "value")
)
def update_plot(extracted_targets, sensor, slider_idx):
    if not extracted_targets or not sensor:
        return go.Figure()

    times = extracted_targets[sensor]["_times"]

    if slider_idx >= len(times):
        return go.Figure()

    current_time = times[slider_idx]
    extracted_target_i = extracted_targets[sensor][current_time]

    pixels = extracted_target_i["pixels"]
    x = [p[0] for p in pixels]
    y = [p[1] for p in pixels]

    fig = go.Figure(
        data=go.Scattergl(
            # MORE ELOQUENT WAY TO INDEX THE PIXELS
            x=x,
            y=y,
            mode="markers",
            marker=dict(size=4, color="red")
        )
    )

    fig.update_layout(
        title=f"Sensor: {sensor} | Time: {current_time}",
        xaxis=dict(
            range=X_RANGE,
            title="X",
            scaleanchor="y"
        ),
        yaxis=dict(
            range=Y_RANGE,
            title="Y",
            autorange="reversed"
        ),
        height=700
    )

    return fig


# ------------------------
# Run
# ------------------------
if __name__ == "__main__":
    app.run(debug=True)
