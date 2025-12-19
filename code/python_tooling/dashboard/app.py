import os
from dash import Dash, dcc, html, Input, Output, no_update
import plotly.graph_objects as go

# Your custom import
from database.load_extracted_targets import load_all_extracted_targets

# ------------------------
# Configuration
# ------------------------
DB_DIR = "../../test_data/"
X_RANGE = [0, 512]
Y_RANGE = [0, 512]

# ------------------------
# Helpers
# ------------------------
def list_databases():
    if not os.path.exists(DB_DIR):
        return []
    return sorted([f for f in os.listdir(DB_DIR) if f.endswith(".db3")])

def extracted_targets_to_records(target_map):
    """Convert custom objects to dicts of X/Y arrays ready for clientside use."""
    out = {}
    for sensor, time_map in target_map.items():
        out[sensor] = {}
        sorted_ts = sorted(time_map.keys())
        for timestamp in sorted_ts:
            target = time_map[timestamp]
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
        html.H2("2D Feature Viewer (Client-side Slider)"),

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
            updatemode="drag",
            tooltip={"always_visible": True, "placement": "bottom"}
        ),

        # Store all data for clientside use
        dcc.Store(id="global-data-store"),

        dcc.Graph(
            id="pixel-plot",
            figure=go.Figure(
                data=[go.Scattergl(x=[], y=[], mode="markers",
                                   marker=dict(size=4, color="red"))],
                layout=go.Layout(
                    xaxis=dict(range=X_RANGE, title="X", scaleanchor="y", scaleratio=1),
                    yaxis=dict(range=Y_RANGE, title="Y", autorange="reversed"),
                    width=600, height=600,
                    template="plotly_white",
                    uirevision="constant"  # preserve zoom/pan
                )
            )
        )
    ]
)

# ------------------------
# Server-side Callbacks
# ------------------------

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
    Output("time-slider", "value"),
    Input("db-dropdown", "value")
)
def load_database(db_file):
    if not db_file:
        return None, [], None, 0, 0

    # Load database and convert to X/Y arrays
    raw_data = load_all_extracted_targets(os.path.join(DB_DIR, db_file))
    dataset = extracted_targets_to_records(raw_data)

    # Sensor dropdown options
    sensors = [s for s in dataset.keys() if s != "_times"]
    if not sensors:
        return None, [], None, 0, 0
    first_sensor = sensors[0]
    num_times = len(dataset[first_sensor]["_times"])

    sensor_options = [{"label": s, "value": s} for s in sensors]

    return dataset, sensor_options, first_sensor, num_times - 1, 0

# ------------------------
# Clientside Callback for slider
# ------------------------

app.clientside_callback(
    """
    function(slider_idx, sensor, data_store) {
        if (!data_store || !sensor) { return window.dash_clientside.no_update; }
        const times = data_store[sensor]["_times"];
        if (slider_idx >= times.length) { return window.dash_clientside.no_update; }

        const current_time = times[slider_idx];
        const frame = data_store[sensor][current_time];

        return {
            data: [{
                x: frame["x"],
                y: frame["y"],
                mode: "markers",
                marker: {size: 4, color: "red"}
            }],
            layout: {title: `Sensor: ${sensor} | Time: ${current_time}`}
        };
    }
    """,
    Output("pixel-plot", "figure"),
    Input("time-slider", "value"),
    Input("sensor-dropdown", "value"),
    Input("global-data-store", "data")
)

# ------------------------
# Run
# ------------------------
if __name__ == "__main__":
    app.run(debug=True)
