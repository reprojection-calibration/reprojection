import os
from dash import Dash, dcc, html, Input, Output, State, no_update
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
        html.H2("2D Feature Viewer (with Play/Stop)"),

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

        html.Div(
            style={"display": "flex", "alignItems": "center", "gap": "20px", "marginBottom": "10px"},
            children=[
                html.Button("▶ Play", id="play-btn", n_clicks=0, style={"width": "100px"}),
                html.Div([
                    html.Label("Time Step:"),
                    dcc.Slider(
                        id="time-slider",
                        min=0, max=0, step=1, value=0,
                        updatemode="drag",
                        tooltip={"always_visible": True, "placement": "bottom"}
                    ),
                ], style={"flexGrow": 1})
            ]
        ),

        # Interval component for the "Play" functionality (disabled by default)
        dcc.Interval(id="play-interval", interval=20, n_intervals=0, disabled=True),

        dcc.Store(id="global-data-store"),

        dcc.Graph(
            id="pixel-plot",
            figure=go.Figure(
                layout=go.Layout(
                    xaxis=dict(range=[0,512], scaleanchor="y", scaleratio=1),
                    yaxis=dict(range=[0,512]),
                    template="plotly_white",
                    uirevision="constant"
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

# UPDATED: Removed Output("time-slider", "value") from here
# to avoid the duplicate output error.
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

    raw_data = load_all_extracted_targets(os.path.join(DB_DIR, db_file))
    dataset = extracted_targets_to_records(raw_data)

    sensors = [s for s in dataset.keys() if s != "_times"]
    if not sensors:
        return None, [], None, 0

    first_sensor = sensors[0]
    num_times = len(dataset[first_sensor]["_times"])
    sensor_options = [{"label": s, "value": s} for s in sensors]

    return dataset, sensor_options, first_sensor, num_times - 1

# ------------------------------------------------
# Play/Stop Logic (Toggles Interval and Button Label)
# ------------------------------------------------
@app.callback(
    Output("play-interval", "disabled"),
    Output("play-btn", "children"),
    Input("play-btn", "n_clicks"),
    State("play-interval", "disabled"),
    prevent_initial_call=True
)
def toggle_play(n_clicks, is_disabled):
    if is_disabled:
        return False, "■ Stop"
    return True, "▶ Play"

# ------------------------------------------------
# Clientside Callback: Auto-advance Slider & Update Plot
# ------------------------------------------------
app.clientside_callback(
    """
    function(n_intervals, slider_val, sensor, data_store, fig, interval_disabled) {
        const ctx = dash_clientside.callback_context;
        const triggered_id = ctx.triggered.length > 0 ? ctx.triggered[0].prop_id : "";
        
        if (!data_store || !sensor || !data_store[sensor]) {
            return [0, window.dash_clientside.no_update];
        }

        let next_slider_val = slider_val;
        const times = data_store[sensor]["_times"];
        const max_idx = times.length - 1;

        // Logic: Reset slider on DB change or increment on Interval
        if (triggered_id.includes("global-data-store.data")) {
            next_slider_val = 0;
        } 
        else if (triggered_id.includes("play-interval.n_intervals") && !interval_disabled) {
            next_slider_val = (slider_val >= max_idx) ? 0 : slider_val + 1;
        }

        const current_time = times[next_slider_val];
        const frame = data_store[sensor][current_time];

        return [
            next_slider_val, 
            {
                data: [{
                    x: frame["x"],
                    y: frame["y"],
                    type: "scattergl",
                    mode: "markers",
                    marker: {size: 4, color: "red"}
                }],
                layout: {
                    ...fig.layout,
                    // FORCE FIXED RANGES HERE
                    xaxis: {
                        range: [0, 512], 
                        title: "X", 
                        scaleanchor: "y", 
                        scaleratio: 1, 
                        autorange: false
                    },
                    yaxis: {
                        range: [0, 512], 
                        title: "Y", 
                        autorange: false
                    },
                    width: 512,
                    height: 512,
                    uirevision: "constant",
                    title: `Sensor: ${sensor} | Time: ${current_time}`
                }
            }
        ];
    }
    """,
    Output("time-slider", "value"),
    Output("pixel-plot", "figure"),
    Input("play-interval", "n_intervals"),
    Input("time-slider", "value"),
    Input("sensor-dropdown", "value"),
    Input("global-data-store", "data"),
    Input("pixel-plot", "figure"),
    State("play-interval", "disabled")
)

if __name__ == "__main__":
    app.run(debug=True)