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

X_RANGE = [0, 1920]
Y_RANGE = [0, 1080]


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
            tooltip={"always_visible": True}
        ),

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


# Populate sensors + slider when DB changes
@app.callback(
    Output("sensor-dropdown", "options"),
    Output("sensor-dropdown", "value"),
    Output("time-slider", "max"),
    Output("time-slider", "marks"),
    Output("time-slider", "value"),
    Input("db-dropdown", "value")
)
def update_db_dependent_controls(db_file):
    if not DB_DIR + db_file:
        return [], None, 0, {}, 0

    db = load_all_extracted_targets(DB_DIR + db_file)
    sensors = ["/cam0/image_raw", "/cam1/image_raw"]
    times = db["/cam0/image_raw"].keys()

    sensor_options = [{"label": s, "value": s} for s in sensors]
    marks = {i: str(t) for i, t in enumerate(times)}

    return sensor_options, sensors[0], len(times) - 1, marks, 0


# Update plot
@app.callback(
    Output("pixel-plot", "figure"),
    Input("db-dropdown", "value"),
    Input("sensor-dropdown", "value"),
    Input("time-slider", "value")
)
def update_plot(db_file, sensor, slider_idx):
    if not DB_DIR + db_file or not sensor:
        return go.Figure()

    df = load_all_extracted_targets(DB_DIR + db_file)
    times = df[TIME_COL].unique()

    if slider_idx >= len(times):
        return go.Figure()

    current_time = times[slider_idx]
    frame_df = df[
        (df[TIME_COL] == current_time) &
        (df[SENSOR_COL] == sensor)
        ]

    fig = go.Figure(
        data=go.Scattergl(
            x=frame_df[X_COL],
            y=frame_df[Y_COL],
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
