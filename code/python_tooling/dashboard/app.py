import os
from dash import callback, Dash, dcc, html, Input, Output, State
import plotly.graph_objects as go

from plot_pose_figures import extract_timestamps_and_poses, plot_pose_figure

from database.load_camera_calibration_data import get_camera_calibration_data_statistics, \
    get_indexable_timestamp_record, load_camera_calibration_data

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
            html.Div(
                children=[
                    html.Div(
                        children=[
                            html.Label(
                                children='Load',
                            ),
                            dcc.Dropdown(
                                id='database-dropdown',
                                placeholder='Select a database',
                                style={'width': '300px'},
                            ),
                            html.Button(
                                children='Refresh Database List',
                                id='refresh-database-list-button',
                                n_clicks=0,
                            ),
                        ],
                        style={'align-items': 'center',
                               'display': 'flex',
                               'flex-direction': 'row',
                               'gap': '10px',
                               'margin': '10px',
                               'flex': '1', },
                    ),
                    html.Div(
                        children=[
                            html.Label(
                                children='Select',
                            ),
                            dcc.Dropdown(
                                id='sensor-dropdown',
                                placeholder='Select a camera sensor',
                                style={'width': '300px'},
                            ),
                        ],
                        style={'align-items': 'center',
                               'display': 'flex',
                               'flex-direction': 'row',
                               'gap': '10px',
                               'margin': '10px',
                               'flex': '1', },
                    ),

                ],
                style={'align-items': 'flex-start',
                       'display': 'flex',
                       'flex-direction': 'column',
                       'gap': '10px',
                       'margin': '10px',
                       'flex': '1', },
            ),

            html.Div(
                children=[
                    html.Div(
                        [
                            html.Div(id="statistics-display"),
                        ]
                    ),
                ],
                style={'align-items': 'top',
                       'display': 'flex',
                       'flex-direction': 'row',
                       'gap': '10px',
                       'margin': '10px',
                       'flex': '1', },
            ),
            html.Div(
                children=[
                    dcc.RadioItems(
                        id="pose-type-selector",
                        options=[
                            {"label": "Initial", "value": "initial"},
                            {"label": "Optimized", "value": "optimized"},
                        ],
                        value="initial",  # default selection
                    )
                ],
                style={'align-items': 'top',
                       'display': 'flex',
                       'flex-direction': 'row',
                       'gap': '10px',
                       'margin': '10px',
                       'flex': '1', },
            ),
        ],
        style={'align-items': 'top',
               'display': 'flex',
               'flex-direction': 'row',
               'gap': '10px',
               'margin': '10px', },

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
                html.Div(id="slider-timestamp", style={"marginTop": "4px"}),
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

    # NOTE(Jack): What we want to prevent is that big chunks of data get sent to and from the browse more than they
    # need to. As the calibration data might be 10, 30, or even 100mb it is important to make sure we only send that to
    # the browser when we need to. Therefore, we designed these two data stores, one heavy one (raw-data-store) and one
    # light one (processed-data-store). In the light one we should find all the metadata required to parameterize and
    # build most of the dashboard (ex. timestamps, number of frames etc.) and the heavy one we find the entire dataset
    # which we actually need to process to build our figures. Unless you absolutely need the raw data you should only
    # use the processed data!
    dcc.Store(id='raw-data-store'),
    dcc.Store(id='processed-data-store'),
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
    Output('raw-data-store', 'data'),
    Output('processed-data-store', 'data'),
    Input('database-dropdown', 'value')
)
def load_database_to_store(db_file):
    if not db_file:
        return None

    db_path = DB_DIR + db_file
    if not os.path.isfile(db_path):
        return None

    raw_data = load_camera_calibration_data(db_path)
    statistics = get_camera_calibration_data_statistics(raw_data)
    indexable_timestamps = get_indexable_timestamp_record(raw_data)

    # TODO(Jack): visualize the statistics in a data panel!
    return raw_data, [statistics, indexable_timestamps]


@callback(
    Output('sensor-dropdown', 'options'),
    Output('sensor-dropdown', 'value'),
    Input('processed-data-store', 'data')
)
def refresh_sensor_list(data):
    if not data:
        return [], None

    statistics, _ = data

    # We use a set here (e.g. the {} brackets) to enforce uniqueness
    sensor_names = sorted(list({sensor_name for sensor_name in statistics.keys()}))
    if len(sensor_names) == 0:
        return [], ''

    return sensor_names, sensor_names[0]


# NOTE(Jack): We want the user to be able to understand intuitively the time relationships of all components. Therefore
# instead of only displaying the frame index or stamp in nanoseconds we add "seconds from start" tick marks to the
# slider.
# TODO(Jack): Can we use one single function to calculate the marks for all figures? At time of writing I think it is
#  just a coincidence that the pose figure markers match this every five second tempo. We should do this
#  programmatically if it helps us drive uniformity of time representation across different plots.
@app.callback(
    Output("frame-id-slider", "marks"),
    Input("sensor-dropdown", "value"),
    State("processed-data-store", "data"),
)
def calculate_slider_ticks(selected_sensor, data):
    _, indexable_timestamps = data
    timestamps_i = indexable_timestamps[selected_sensor]

    t0 = timestamps_i[0]
    seconds_list = [(t - t0) / 1e9 for t in timestamps_i]

    max_time = seconds_list[-1]
    # TODO(Jack): This line here hardcodes the function to generate marks with a spacing of 5s. For long datasets this
    #  can get cramped! Figure a intelligent way to configure this. At time of writing the 5s interval matched the pose
    #  pose figure interval, but this is not programmatically set!
    target_times = range(0, int(max_time) + 1, 5)

    marks = {}
    for target in target_times:
        closest_index = None
        smallest_error = float("inf")

        for i, seconds in enumerate(seconds_list):
            error = abs(seconds - target)
            if error < smallest_error:
                smallest_error = error
                closest_index = i

        # WARN(Jack): This is not a critical warning, but note that in this function we are introducing some
        # approximation error. Yes we are picking the closest index/timestamp to the target time, but it is not exactly
        # the same time. I do not know where this might cause problems one day, but we should not forget this
        # possibility!
        marks[closest_index] = f"{target}s"

    return marks


# TODO(Jack): We need to display the exact nanosecond timestamp of the current frame somewhere and somehow. If this is
#  is the best way to do this I am not 100% sure just yet.
app.clientside_callback(
    """
    function(frame_idx, data, sensor) {
         if (!data || !sensor) {
            return "";
        }
        
        const timestamps = data[1][sensor]
        if (!timestamps || timestamps.length == 0 || timestamps.length <= frame_idx){
            return "";
        }
        
        const timestamp_i = BigInt(timestamps[frame_idx])

        return "t = " + timestamp_i.toString() + " ns";
    }
    """,
    Output("slider-timestamp", "children"),
    Input("frame-id-slider", "value"),
    State("processed-data-store", "data"),
    State("sensor-dropdown", "value"),
)


# Callback to update the loaded statuses
@app.callback(
    Output("statistics-display", "children"),
    Input("sensor-dropdown", "value"),
    State("processed-data-store", "data"),
)
def update_statistics(selected_sensor, data):
    statistics, _ = data

    return [
        html.Div(
            [
                html.Div(
                    "",
                    style={
                        "width": "20px", "height": "20px", "backgroundColor": "green" if value != 0 else "red",
                        "display": "inline-block", "margin-right": "10px"
                    },
                ),
                html.Div(value, style={"width": "35px", "display": "inline-block"}),
                html.Div(key, style={"width": "200px", "display": "inline-block"}),
            ],
        )
        for key, value in statistics[selected_sensor].items()
    ]


@callback(
    Output('rotation-graph', 'figure'),
    Output('translation-graph', 'figure'),
    Input('sensor-dropdown', 'value'),
    State('raw-data-store', 'data'),
)
def update_translation_graph(selected_sensor, raw_data):
    if not selected_sensor or not raw_data:
        return {}, {}

    # TODO(Jack): Technically this is not nice. To access a key (frames) without checking that it exists. However this
    # is so fundamental to the data structure I think I can be forgiven for this. Remove this todo later if it turns out
    # to be a nothing burger.
    frames = raw_data[selected_sensor]['frames']
    if frames is None:
        return {}, {}

    # TODO WARN ERROR
    # TODO TODO TODO
    # TODO(Jack): Add optimized pose initialization! Or the option to choose between the two.
    # TODO TODO TODO
    # TODO(Jack): Make sure we are not recalculating any timestamp related thing! One thing here that means it is maybe
    #  not required to use the indexed timestamps is that here we depend only on correspondence and not order to plot
    #  these figures.
    timestamps, data = extract_timestamps_and_poses(frames, 'initial')

    rotations = [d[:3] for d in data]
    rot_fig = plot_pose_figure(timestamps, rotations, 'Orientation', 'Axis Angle (rad)', x_name='rx', y_name='ry',
                               z_name='rz')

    translations = [d[3:] for d in data]
    trans_fig = plot_pose_figure(timestamps, translations, 'Translation', 'Meter (m)')

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
    State("processed-data-store", "data"),
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
    statistics, _ = data
    n_frames = statistics[sensor]['total_frames']

    return xy_fig, pixel_fig, max(n_frames - 1, 0)


app.clientside_callback(
    """
    function(frame_idx, raw_data, processed_data, sensor, xy_fig, pixel_fig) {
        if (!raw_data || !processed_data || !sensor || !xy_fig || !pixel_fig) {
              console.log("One or more of the inputs is missing.");
            return [xy_fig, pixel_fig];
        }
        
        const timestamps = processed_data[1][sensor]
        if (!timestamps || timestamps.length == 0 || timestamps.length <= frame_idx){
              console.log("Invalid timestamps or frame index out of bounds:", sensor);
            return [xy_fig, pixel_fig];
        }
        
        const timestamp_i = BigInt(timestamps[frame_idx])
        if (!raw_data[sensor] || !raw_data[sensor]['frames'] || !raw_data[sensor]['frames'][timestamp_i]) {
              console.log("Raw data structure is incomplete for sensor:", sensor);
            return [xy_fig, pixel_fig];
        }
        
        const extracted_target = raw_data[sensor]['frames'][timestamp_i].extracted_target
        if (!extracted_target) {
            console.log("No extracted_target found at frame:", frame_idx, "for sensor:", sensor);
            return [xy_fig, pixel_fig];
        }

        // NOTE(Jack): Dash will only update the graph if it gets a new figure. If you only mutate the input figure then
        // the reference/address/figure id is the same and dash will think you simply returned the same figure, and 
        // not re-render it. Therefore we need to actually create a new figure so that Dash is forced to re-render it 
        // with the new points we add below. Here we use these JSON helper functions to create a copy of the figure 
        // which when returned will be recognized by Dash as a new figure which needs to be rendered. There are several 
        // issues on github (ex. https://github.com/plotly/dash/issues/1040) with people confused by this. 
        xy_fig = JSON.parse(JSON.stringify(xy_fig));
        const pts = extracted_target.points;
        xy_fig.data[0].x = pts.map(p => p[0]);
        xy_fig.data[0].y = pts.map(p => p[1]);

        pixel_fig = JSON.parse(JSON.stringify(pixel_fig));
        const pix = extracted_target.pixels;
        pixel_fig.data[0].x = pix.map(p => p[0]);
        pixel_fig.data[0].y = pix.map(p => p[1]);
        
        // If reprojection errors are available we will color the points and pixels according to them. If not available
        // simply return the figures with the plain colored points and pixels.
        const reprojection_errors = raw_data[sensor]['frames'][timestamp_i]['reprojection_errors']
        if (!reprojection_errors) {
            // If no reprojection errors are available then return to default marker configuration.
            xy_fig.data[0].marker = {size: 6}
            pixel_fig.data[0].marker = {size: 6}
            
            return [xy_fig, pixel_fig];
        }
        
        // TODO WARN ERROR
        // TODO WARN ERROR
        // TODO WARN ERROR
        // TODO WARN ERROR HANDLE INTIIAL VS OPTIMIZED!!! For now we just hardcode initial.
        const initial_reprojection_error = reprojection_errors['optimized']
        if (!initial_reprojection_error) {
            return [xy_fig, pixel_fig];
        }
        
        const markers = {size: 6, 
                        color: initial_reprojection_error.map(p => Math.sqrt(p[0] * p[0] + p[1] * p[1])), 
                        colorscale: "Bluered", 
                        cmin: 0, cmax: 1};
        xy_fig.data[0].marker = markers
        pixel_fig.data[0].marker = markers
        
        return [xy_fig, pixel_fig];
    }
    """,
    Output("targets-xy-graph", "figure"),
    Output("targets-pixels-graph", "figure"),
    Input("frame-id-slider", "value"),
    State("raw-data-store", "data"),
    State("processed-data-store", "data"),
    State("sensor-dropdown", "value"),
    State("targets-xy-graph", "figure"),
    State("targets-pixels-graph", "figure"),
)

if __name__ == '__main__':
    app.run(debug=True)
