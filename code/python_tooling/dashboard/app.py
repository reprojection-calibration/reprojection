from dash import Dash, dcc, html, Input, Output, State, callback
import os
from database.load_extracted_targets import load_all_extracted_targets, split_extracted_targets_by_sensor
from database.load_calibration_poses import load_calibration_poses
from plot_pose_figures import plot_rotation_figure, plot_translation_figure

# TODO(Jack): Do not hardcode this
DB_DIR = '../../test_data/'

app = Dash(title='Reprojection', update_title=None)
app.layout = html.Div([
    html.H2("Reprojection - the future is calibrated."),

    html.Div(
        children=[
            html.Label('Load'),
            dcc.Dropdown(id='database-dropdown', placeholder="Select a database", style={"width": "50%"}),
            html.Button('Refresh Database List', id='refresh-database-list-button', n_clicks=0),
        ],
        style={"display": "flex", "gap": "10px", "marginBottom": "20px"},
    ),

    html.Div(
        children=[
            html.Label('Select'),
            dcc.Dropdown(id='sensor-dropdown', placeholder="Select a camera sensor", style={"width": "50%"}),
        ],
        style={"display": "flex", "gap": "10px", "marginBottom": "20px"},
    ),

    dcc.Tabs([
        dcc.Tab(label='Poses', children=[
            dcc.Graph(id='rotation-graph'),
            dcc.Graph(id='translation-graph'),
        ]),
        dcc.Tab(label='Feature Extraction',
                children=[
                    dcc.Slider(
                        id="targets-frame-slider",
                        min=0,
                        max=0,
                        step=1,
                        value=0,
                        updatemode="drag",
                        marks=None,
                        tooltip={"placement": "bottom", "always_visible": True},
                    ),
                    html.Div(
                        style={"display": "flex", "gap": "20px"},
                        children=[
                            dcc.Graph(id="targets-xy-graph", style={"width": "50%"}),
                            dcc.Graph(id="targets-pixels-graph", style={"width": "50%"}),
                        ]
                    ),
                    html.Button("▶ Play", id="play-button", n_clicks=0),
                ]),
    ]),

    dcc.Interval(
        id="play-interval",
        interval=50,
        disabled=True,
    ),

    dcc.Store(id='database-store'),
])


@callback(
    Output('database-dropdown', 'options'),
    Output('database-dropdown', 'value'),
    Input('refresh-database-list-button', 'n_clicks')
)
def refresh_database_list(_):
    if not os.path.exists(DB_DIR):
        return [], ''

    database_names = sorted([f for f in os.listdir(DB_DIR) if f.endswith(".db3")])
    if len(database_names) == 0:
        return [], ''

    return database_names, database_names[0]


@callback(
    Output("play-interval", "disabled"),
    Output("play-button", "children"),
    Input("play-button", "n_clicks"),
    prevent_initial_call=True,
)
def toggle_play(n_clicks):
    playing = n_clicks % 2 == 1
    return not playing, "⏸ Pause" if playing else "▶ Play"


@callback(
    Output("targets-frame-slider", "value"),
    Input("play-interval", "n_intervals"),
    State("targets-frame-slider", "value"),
    State("targets-frame-slider", "max"),
    prevent_initial_call=True,
)
def advance_slider(_, value, max_value):
    if value is None:
        return 0
    if value >= max_value:
        return 0  # loop playback
    return value + 1


@callback(
    Output('database-store', 'data'),
    Input('database-dropdown', 'value')
)
def load_database_to_store(db_file):
    if not db_file:
        return None

    full_path = DB_DIR + db_file
    if not os.path.isfile(full_path):
        return None

    df_initial, df_optimized, df_external = load_calibration_poses(full_path)

    # Make pandas data frame json serializable which is required for anything sent to the browser in dash
    data = {'poses': {'initial': df_initial.to_dict('records'), 'optimized': df_optimized.to_dict('records'),
                      'external': df_external.to_dict('records')}}

    df_extracted_targets = load_all_extracted_targets(full_path)
    data['extracted_targets'] = split_extracted_targets_by_sensor(df_extracted_targets)

    return data


@callback(
    Output('sensor-dropdown', 'options'),
    Output('sensor-dropdown', 'value'),
    Input('database-store', 'data')
)
def refresh_sensor_list(data):
    if not data:
        return [], None

    # TODO(Jack): We should get the sensor name from the images or extracted targets, as they are more likely to be there than the initial poses!
    # We use a set here (e.g. the {} brackets) to enforce uniqueness
    sensor_names = sorted(list({row['sensor_name'] for row in data['poses']['initial']}))
    if len(sensor_names) == 0:
        return [], ''

    # Set the default dropdown sensor value to the first sensor
    return sensor_names, sensor_names[0]


@callback(
    Output('rotation-graph', 'figure'),
    Output('translation-graph', 'figure'),
    Input('sensor-dropdown', 'value'),
    State('database-store', 'data'),
)
def update_translation_graph(selected_sensor, data):
    if not selected_sensor or not data:
        return {}, {}

    initial_pose = sorted([sensor for sensor in data['poses']['initial'] if sensor['sensor_name'] == selected_sensor],
                          key=lambda x: x['timestamp_ns'])

    rot_fig = plot_rotation_figure(initial_pose, legendgroup='Initial', marker='x')
    trans_fig = plot_translation_figure(initial_pose, legendgroup='Initial', marker='x')

    if data['poses']['external'] is not None:
        gt_poses = sorted([sensor for sensor in data['poses']['external']], key=lambda x: x['timestamp_ns'])
        rot_fig = plot_rotation_figure(gt_poses, fig=rot_fig, legendgroup='External')
        trans_fig = plot_translation_figure(gt_poses, fig=trans_fig, legendgroup='External')

    return rot_fig, trans_fig


@callback(
    Output("targets-xy-graph", "figure", allow_duplicate=True),
    Output("targets-pixels-graph", "figure", allow_duplicate=True),
    Output("targets-frame-slider", "max"),
    Input("sensor-dropdown", "value"),
    State("database-store", "data"),
    prevent_initial_call=True,
)
def init_2d_target_figures(sensor, data):
    if not sensor or not data:
        return {}, {}, 0

    frames = data["extracted_targets"].get(sensor, [])
    n_frames = len(frames)

    # World XY figure
    xy_fig = {
        "data": [{
            "type": "scatter",
            "mode": "markers",
            "x": [],
            "y": [],
            "marker": {"size": 6},
        }],
        "layout": {
            "title": "Target points (XY)",
            "xaxis": {"range": [0, 1],
                      "title": "X",
                      "constrain": "domain",
                      "scaleanchor": "y",
                      },
            "yaxis": {"range": [0, 1],
                      "title": "Y",
                      },
        }
    }

    pixel_fig = {
        "data": [{
            "type": "scatter",
            "mode": "markers",
            "x": [],
            "y": [],
            "marker": {"size": 6},
        }],
        "layout": {
            "title": "Extracted Feature",
            "xaxis": {
                "range": [0, 512],  # ERROR(Jack): Do not hardcode image dimensions!
                "title": "u",
                "constrain": "domain",
            },
            "yaxis": {
                "range": [512, 0],  # invert Y for image coords
                "title": "v",
                "scaleanchor": "x",
            },
        }
    }

    return xy_fig, pixel_fig, max(n_frames - 1, 0)


app.clientside_callback(
    """
    function(frame_idx, data, sensor, xy_fig, pixel_fig) {
        if (!data || !sensor || !xy_fig || !pixel_fig) {
            return [xy_fig, pixel_fig];
        }

        const frames = data.extracted_targets[sensor];
        if (!frames || frames.length === 0) {
            return [xy_fig, pixel_fig];
        }

        const frame = frames[frame_idx];
        if (!frame) {
            return [xy_fig, pixel_fig];
        }

        // TODO(Jack): Is this really required?
        // IMPORTANT: clone figures so Dash detects changes
        xy_fig = JSON.parse(JSON.stringify(xy_fig));
        pixel_fig = JSON.parse(JSON.stringify(pixel_fig));

        const pts = frame.points;
        xy_fig.data[0].x = pts.map(p => p[0]);
        xy_fig.data[0].y = pts.map(p => p[1]);

        const pix = frame.pixels;
        pixel_fig.data[0].x = pix.map(p => p[0]);
        pixel_fig.data[0].y = pix.map(p => p[1]);

        return [xy_fig, pixel_fig];
    }
    """,
    Output("targets-xy-graph", "figure"),
    Output("targets-pixels-graph", "figure"),
    Input("targets-frame-slider", "value"),
    State("database-store", "data"),
    State("sensor-dropdown", "value"),
    State("targets-xy-graph", "figure"),
    State("targets-pixels-graph", "figure"),
)

if __name__ == '__main__':
    app.run(debug=True)
