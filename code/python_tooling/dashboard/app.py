import os
from dash import callback, Dash, dcc, html, Input, Output, State
import plotly.graph_objects as go

from plot_pose_figures import plot_rotation_figure, plot_translation_figure

from database.load_extracted_targets import load_extracted_targets_df, split_extracted_targets_by_sensor
from database.load_poses import load_calibration_poses
from database.load_images import load_images_df, split_images_by_sensor

# TODO(Jack): Do not hardcode this - giving a user the ability to interact with the file system in a gui is not so
#  trivial but can be done with some tk tools or other libraries
DB_DIR = '../../test_data/'
# TODO(Jack): Place meta data like this in config file or in database. For now we use globals...
IMAGE_DIMENSIONS = (512, 512)

# NOTE(Jack): If we do not specify the title and update behavior update here the browser tab will constantly and
# annoyingly show "Updating..." constantly.
app = Dash(title='Reprojection', update_title=None)

app.layout = html.Div([
    html.H2('Reprojection - The future is calibrated.'),

    html.Div(
        children=[
            html.Label(
                children='Load',
            ),
            dcc.Dropdown(
                id='database-dropdown',
                placeholder='Select a database',
                style={'width': '50%'},
            ),
            html.Button(
                children='Refresh Database List',
                id='refresh-database-list-button',
                n_clicks=0,
            ),
        ],
        style={'display': 'flex', 'gap': '10px', 'marginBottom': '20px'},
    ),

    html.Div(
        children=[
            html.Label(
                children='Select',
            ),
            dcc.Dropdown(
                id='sensor-dropdown',
                placeholder='Select a camera sensor',
                style={'width': '50%'},
            ),
        ],
        style={'display': 'flex', 'gap': '10px', 'marginBottom': '20px'},
    ),

    dcc.Tabs([
        dcc.Tab(
            children=[
                dcc.Graph(
                    id='rotation-graph',
                ),
                dcc.Graph(
                    id='translation-graph',
                ),
            ],
            label='Camera Poses',
        ),
        dcc.Tab(
            children=[
                # TODO(Jack): We should have one slider/play button at the top level, not in any specific tab or section
                #  that drives all related animations.
                # TODO(Jack): Once the data is loaded and we know the real number of frames we should either display
                #  some frame ids and or timestamps along the slider so the user can have some intuition of time and
                #  scale.
                dcc.Slider(
                    id="frame-id-slider",
                    marks=None,
                    min=0, max=0, step=1, value=0,
                    tooltip={"placement": "bottom", "always_visible": True},
                    updatemode="drag",
                ),
                html.Div(
                    children=[
                        dcc.Graph(
                            id="targets-xy-graph",
                            style={"width": "50%"}
                        ),
                        dcc.Graph(
                            id="targets-pixels-graph",
                            style={"width": "50%"}
                        ),
                    ],
                    style={'display': 'flex', 'gap': '10px', 'marginBottom': '20px'},
                ),
                # The animation plays by default therefore the button is initialized with the pause graphic
                html.Button(
                    children="⏸ Pause",
                    id="play-button",
                    n_clicks=0,
                ),
            ],
            label='Feature Extraction',
        ),
    ]),

    # Components without a visual representation are found here at the bottom (ex. Interval, Store etc.)
    dcc.Interval(
        disabled=False,
        id="play-interval",
        interval=50,
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
    Output('database-store', 'data'),
    Input('database-dropdown', 'value')
)
def load_database_to_store(db_file):
    if not db_file:
        return None

    full_path = DB_DIR + db_file
    if not os.path.isfile(full_path):
        return None

    data = {}

    # WARN(Jack): This has only been tested with databases where the images were not there! When we actually have a
    # database with images we can first just ignore them explicitly, and then eventually figure out a visualization
    # strategy. This means that the image "data" here is expected to be null.
    df_images = load_images_df(full_path)
    data['images'] = split_images_by_sensor(df_images)

    # Use .to_dict('records') to make pandas data frame json serializable, which is required for anything sent to the
    # browser in dash.
    df_initial, df_optimized, df_external = load_calibration_poses(full_path)
    data['poses'] = {'initial': df_initial.to_dict('records'), 'optimized': df_optimized.to_dict('records'),
                     'external': df_external.to_dict('records')}

    df_extracted_targets = load_extracted_targets_df(full_path)
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

    # We use a set here (e.g. the {} brackets) to enforce uniqueness
    sensor_names = sorted(list({sensor_name for sensor_name in data['images'].keys()}))
    if len(sensor_names) == 0:
        return [], ''

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

    # The initial pose in the context of calibration is the pose calculated via DLT and/or PNP that is used as the input
    # for the full nonlinear optimization later. It is important, but also just an intermediate output on the path to
    # full calibration
    initial_poses = sorted([sensor for sensor in data['poses']['initial'] if sensor['sensor_name'] == selected_sensor],
                           key=lambda x: x['timestamp_ns'])

    # TODO(Jack): Figure out a way to display the legend group name in the legend so that people actually know what they
    #  are looking at.
    rot_fig = plot_rotation_figure(initial_poses, legendgroup='Initial', marker='x')
    trans_fig = plot_translation_figure(initial_poses, legendgroup='Initial', marker='x')

    # TODO(Jack): Should we give the user the option to explicitly toggle the external poses? Or should we always
    #  display them anyway? For most real world data there will be no external pose.
    if data['poses']['external'] is not None:
        external_poses = sorted([sensor for sensor in data['poses']['external']], key=lambda x: x['timestamp_ns'])

        rot_fig = plot_rotation_figure(external_poses, fig=rot_fig, legendgroup='External')
        trans_fig = plot_translation_figure(external_poses, fig=trans_fig, legendgroup='External')

    return rot_fig, trans_fig


@callback(
    Output("play-interval", "disabled"),
    Output("play-button", "children"),
    Input("play-button", "n_clicks"),
)
def toggle_play(n_clicks):
    # We play by default (n_clicks=0), which means that only when we have an odd number of clicks is "playing" false.
    playing = n_clicks % 2 == 0

    # If we want to play then we do NOT want the play interval disabled and we want the button to display the pause
    # graphic and message so the user knows to click there to pause. If we are already paused then the opposite is true.
    if playing:
        return False, "⏸ Pause"
    else:
        return True, "▶ Play"


@callback(
    Output("frame-id-slider", "value"),
    Input("play-interval", "n_intervals"),
    State("frame-id-slider", "value"),
    State("frame-id-slider", "max"),
)
def advance_slider(_, value, max_value):
    if value is None:
        return 0
    if value >= max_value:
        return 0  # loop playback

    return value + 1


# TODO(Jack): Technically we only need the sensor name to get the frame-id-slider output, but this does not necessarily
#  have anything to do with configuring the figures initially. It might make sense to move the frame id slider
#  dependency to another place that is more related or independent than here.
@callback(
    Output("targets-xy-graph", "figure", allow_duplicate=True),
    Output("targets-pixels-graph", "figure", allow_duplicate=True),
    Output("frame-id-slider", "max"),
    Input("sensor-dropdown", "value"),
    State("database-store", "data"),
    prevent_initial_call=True,
)
def init_extracted_target_figures(sensor, data):
    if not sensor or not data:
        return {}, {}, 0

    # TODO(Jack): Confirm/test ALL axes properties (ranges, names, orders etc.) None of this has been checked! Even the
    #  coordinate conventions of the pixels and points needs to be checked!
    xy_fig = go.Figure()
    xy_fig.add_trace(
        go.Scatter(
            x=[],
            y=[],
            mode="markers",
            marker=dict(size=6),
        )
    )
    xy_fig.update_layout(
        title="Target Points (XY)",
        xaxis=dict(
            range=[-1, 1],  # ERROR(Jack): Do not hardcode or use global
            title=dict(text="x"),
            constrain="domain",
        ),
        yaxis=dict(
            range=[-1, 1],  # ERROR(Jack): Do not hardcode or use global
            title=dict(text="y"),
            scaleanchor="x",
        ),
    )

    pixel_fig = go.Figure()
    pixel_fig.add_trace(
        go.Scatter(
            x=[],
            y=[],
            mode="markers",
            marker=dict(size=6),
            name="Pixels",
        )
    )
    pixel_fig.update_layout(
        title="Extracted Pixel Features",
        xaxis=dict(
            range=[0, IMAGE_DIMENSIONS[0]],  # ERROR(Jack): Do not hardcode or use global
            title=dict(text="u"),
            constrain="domain",
        ),
        yaxis=dict(
            range=[IMAGE_DIMENSIONS[1], 0],  # invert Y for image coords
            title=dict(text="v"),
            scaleanchor="x",
        ),
    )

    # TODO(Jack): Why is this in this method??? See comment at top of function.
    # Get the number of frames to fill the max value of the slider
    frames = data["extracted_targets"].get(sensor, [])
    n_frames = len(frames)

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

        // NOTE(Jack): Dash will only update the graph if it gets a new figure. If you only mutate the input figure then
        // the reference/addrress/figure id is the same and dash will think you simply returned the same figure, and 
        // not re-render it. Therefore we need to actually create a new figure so that Dash is forced to re-render it 
        // with the new points we add below. Here we use these JSON helper functions to create a copy of the figure 
        // which when returned will be recognized by Dash as a new figure which needs to be rendered. There are several 
        // issues on github (ex. https://github.com/plotly/dash/issues/1040) with people confused by this. 
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
    Input("frame-id-slider", "value"),
    State("database-store", "data"),
    State("sensor-dropdown", "value"),
    State("targets-xy-graph", "figure"),
    State("targets-pixels-graph", "figure"),
)

if __name__ == '__main__':
    app.run(debug=True)
