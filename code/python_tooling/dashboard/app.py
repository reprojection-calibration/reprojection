import os
from dash import Dash, dcc, html, Input, Output, Patch, no_update
import plotly.graph_objects as go

# Assuming this is your custom import
from database.load_extracted_targets import load_all_extracted_targets

# ------------------------
# Configuration
# ------------------------
DB_DIR = "../../test_data/"
X_RANGE = [0, 512]
Y_RANGE = [0, 512]

# GLOBAL CACHE: This keeps the data on the server.
# This is the "secret sauce" for speed.
GLOBAL_DATA_STORE = {}

# ------------------------
# Helpers
# ------------------------
def list_databases():
    if not os.path.exists(DB_DIR):
        return []
    return sorted([f for f in os.listdir(DB_DIR) if f.endswith(".db3")])

def extracted_targets_to_records(target_map):
    """Converts custom objects to a dict of lists for fast access."""
    out = {}
    for sensor, time_map in target_map.items():
        out[sensor] = {}
        # Sort timestamps once during load
        sorted_ts = sorted(time_map.keys())
        for timestamp in sorted_ts:
            target = time_map[timestamp]
            # Pre-calculate X and Y to avoid loops during slider movement
            pixels = target.pixels.tolist()
            out[sensor][str(timestamp)] = {
                "x": [p[0] for p in pixels],
                "y": [p[1] for p in pixels]
            }
        out[sensor]["_times"] = [str(ts) for ts in sorted_ts]
    return out

# ------------------------
# Dash App
# ------------------------
app = Dash(__name__)

app.layout = html.Div(
    style={"width": "85%", "margin": "auto", "fontFamily": "sans-serif"},
    children=[
        html.H2("2D Feature Viewer (Optimized)"),

        html.Div(
            style={"display": "flex", "gap": "10px", "marginBottom": "20px"},
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

        dcc.Dropdown(
            id="sensor-dropdown",
            placeholder="Select a sensor",
            style={"marginBottom": "20px"}
        ),

        html.Label("Time Step:"),
        dcc.Slider(
            id="time-slider",
            min=0,
            max=0,
            step=1,
            value=0,
            updatemode="drag",  # Enables continuous updating while sliding
            tooltip={"always_visible": True, "placement": "bottom"}
        ),

        # Store only the filename key, NOT the data itself
        dcc.Store(id="active-db-key"),

        dcc.Graph(
            id="pixel-plot",
            # Pre-configure the figure structure for Patch updates
            figure=go.Figure(
                data=[go.Scattergl(
                    x=[], y=[],
                    mode="markers",
                    marker=dict(size=4, color="red")
                )],
                layout=go.Layout(
                    xaxis=dict(range=X_RANGE, title="X", scaleanchor="y", scaleratio=1),
                    yaxis=dict(range=Y_RANGE, title="Y", autorange="reversed"),
                    width=600, height=600,
                    template="plotly_white",
                    uirevision='constant' # Prevents zoom reset on slider move
                )
            )
        )
    ]
)

# ------------------------
# Callbacks
# ------------------------

@app.callback(
    Output("db-dropdown", "options"),
    Input("reload-btn", "n_clicks")
)
def reload_database_list(_):
    return [{"label": db, "value": db} for db in list_databases()]

@app.callback(
    Output("active-db-key", "data"),
    Input("db-dropdown", "value")
)
def load_db_to_global_cache(db_file):
    if not db_file:
        return None

    # Load and process only if not already in memory
    if db_file not in GLOBAL_DATA_STORE:
        full_path = os.path.join(DB_DIR, db_file)
        raw_data = load_all_extracted_targets(full_path)
        GLOBAL_DATA_STORE[db_file] = extracted_targets_to_records(raw_data)

    return db_file

@app.callback(
    Output("sensor-dropdown", "options"),
    Output("sensor-dropdown", "value"),
    Output("time-slider", "max"),
    Output("time-slider", "value"),
    Input("active-db-key", "data")
)
def update_controls(db_key):
    if not db_key or db_key not in GLOBAL_DATA_STORE:
        return [], None, 0, 0

    data = GLOBAL_DATA_STORE[db_key]
    sensors = [s for s in data.keys()]

    if not sensors:
        return [], None, 0, 0

    first_sensor = sensors[0]
    num_times = len(data[first_sensor]["_times"])

    sensor_options = [{"label": s, "value": s} for s in sensors]
    return sensor_options, first_sensor, num_times - 1, 0

@app.callback(
    Output("pixel-plot", "figure"),
    Input("time-slider", "value"),
    Input("sensor-dropdown", "value"),
    Input("active-db-key", "data"),
    prevent_initial_call=True
)
def update_plot_efficiently(slider_idx, sensor, db_key):
    # If no data is loaded yet, don't do anything
    if not db_key or not sensor or db_key not in GLOBAL_DATA_STORE:
        return no_update

    dataset = GLOBAL_DATA_STORE[db_key]
    times = dataset[sensor]["_times"]

    if slider_idx >= len(times):
        return no_update

    current_time_str = times[slider_idx]
    frame_data = dataset[sensor][current_time_str]

    # Use Patch to update only the specific properties needed
    # This is much faster than recreating the whole Figure object
    patched_fig = Patch()
    patched_fig["data"][0]["x"] = frame_data["x"]
    patched_fig["data"][0]["y"] = frame_data["y"]
    patched_fig["layout"]["title"] = f"Sensor: {sensor} | Time: {current_time_str}"

    return patched_fig

# ------------------------
# Run
# ------------------------
if __name__ == "__main__":
    app.run(debug=True)