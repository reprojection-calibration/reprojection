import os
from dash import callback, Dash, dcc, html, Input, Output, State
import plotly.graph_objects as go
from dash.exceptions import PreventUpdate

from plot_pose_figures import calculate_ticks_from_timestamps, extract_timestamps_and_poses_sorted, plot_pose_figure

from database.load_camera_calibration_data import get_camera_calibration_data_statistics, \
    get_indexable_timestamp_record, load_camera_calibration_data
from database.types import PoseType

# TODO(Jack): Use a css style sheet instead of individually specifying the properties everywhere! This does not scale.
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
                        style={'alignItems': 'center',
                               'display': 'flex',
                               'flexDirection': 'row',
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
                        style={'alignItems': 'center',
                               'display': 'flex',
                               'flexDirection': 'row',
                               'gap': '10px',
                               'margin': '10px',
                               'flex': '1', },
                    ),

                ],
                style={'alignItems': 'flex-start',
                       'display': 'flex',
                       'flexDirection': 'column',
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
                style={'alignItems': 'top',
                       'display': 'flex',
                       'flexDirection': 'row',
                       'gap': '10px',
                       'margin': '10px',
                       'flex': '1', },
            ),
            html.Div(
                children=[
                    dcc.RadioItems(
                        id="pose-type-selector",
                        options=[
                            {"label": "Initial", "value": PoseType.Initial},
                            {"label": "Optimized", "value": PoseType.Optimized},
                        ],
                        value=PoseType.Initial,
                    )
                ],
                style={'alignItems': 'top',
                       'display': 'flex',
                       'flexDirection': 'row',
                       'gap': '10px',
                       'margin': '10px',
                       'flex': '1', },
            ),
        ],
        style={'alignItems': 'top',
               'display': 'flex',
               'flexDirection': 'row',
               'gap': '10px',
               'margin': '10px', },

    ),
    html.Div(
        children=[
            # The animation plays by default therefore the button is initialized with the pause graphic
            html.Button(
                children="⏸ Pause",
                id="play-button",
                n_clicks=0,
                style={
                    "width": "50px",
                },
            ),
            html.Div(
                children=[
                    dcc.Slider(
                        id="frame-id-slider",
                        marks=None,
                        min=0, max=0, step=1, value=0,
                        tooltip={"placement": "top", "always_visible": True},
                        updatemode="drag",
                    ),
                ],
                style={
                    "width": "70%",
                },
            ),
            html.Div(
                children=[
                    html.P("Current timestamp (ns)"),
                    html.Div(
                        id="slider-timestamp",
                    ),
                ],
            ),
        ],
        style={
            'alignItems': 'top',
            'display': 'flex',
            'flexDirection': 'row',
            'gap': '10px',
            'margin': '10px',
            'flex': '1', },
    ),

    dcc.Tabs([
        dcc.Tab(
            children=[
                html.Div(
                    children=[
                        html.Div(
                            children=[
                                html.Label(
                                    children='Max reprojection error (pix)',
                                ),
                                dcc.Input(
                                    id='max-reprojection-error-input',
                                    type='number',
                                    min=0, max=1000,
                                    value=1,
                                )
                            ],
                            style={
                                "display": "flex",
                                "flexDirection": "row",
                                "gap": "20px",
                                "flex": "1",
                                "width": "100%",
                            },
                        ),
                        html.Div(
                            children=[
                                dcc.Graph(
                                    id="targets-xy-graph",
                                    style={
                                        "width": "100%",
                                        "height": "60vh",
                                    },
                                ),
                                dcc.Graph(
                                    id="targets-pixels-graph",
                                    style={
                                        "width": "100%",
                                        "height": "60vh",
                                    },
                                ),
                            ],
                            style={
                                "display": "flex",
                                "flexDirection": "row",
                                "gap": "20px",
                                "flex": "1",
                                "width": "100%",
                            },
                        ),
                    ],
                    style={
                        "display": "flex",
                        "flexDirection": "column",
                        "gap": "20px",
                        "flex": "1",
                        "width": "100%",
                    },
                ),
            ],
            label='Feature Extraction',
        ),
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
    ]),

    # Components without a visual representation are found here at the bottom (ex. Interval, Store etc.)
    dcc.Interval(
        disabled=False,
        id="play-interval",
        interval=100,
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
    dcc.Store(id='translation-figure-store'),
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


@app.callback(
    Output("frame-id-slider", "marks"),
    Input("sensor-dropdown", "value"),
    State("processed-data-store", "data"),
)
def update_slider_marks(selected_sensor, data):
    if selected_sensor is None or data is None:
        raise PreventUpdate

    _, timestamps_sorted = data
    timestamps_ns = timestamps_sorted[selected_sensor]

    tickvals_idx, _, ticktext = calculate_ticks_from_timestamps(timestamps_ns)

    return dict(zip(tickvals_idx, ticktext))


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

        return timestamp_i.toString();
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
    if selected_sensor is None or data is None:
        raise PreventUpdate

    statistics, _ = data

    return [
        html.Div(
            [
                html.Div(
                    "",
                    style={
                        "width": "20px", "height": "20px", "backgroundColor": "green" if value != 0 else "red",
                        "display": "inline-block", "marginRight": "10px"
                    },
                ),
                html.Div(value, style={"width": "35px", "display": "inline-block"}),
                html.Div(key, style={"width": "200px", "display": "inline-block"}),
            ],
        )
        for key, value in statistics[selected_sensor].items()
    ]


# TODO(Jack): Rename to "pose" concept, not just translation
@callback(
    Output("translation-figure-store", "data"),
    Input('sensor-dropdown', 'value'),
    Input('pose-type-selector', 'value'),
    State('raw-data-store', 'data'),
)
def update_translation_graph(selected_sensor, pose_type, raw_data):
    # TODO(Jack): What is an effective way to actually use this mechanism? Having it at the start of every single
    #  callback does not feel right somehow. But managing when each input or state is available is not trivial! For
    #  example here the pose type always should have a default value, and technically the raw-data store should always
    #  be populated before the sensor dropdown triggers (because the sensor dropdown depends on the raw data being
    #  loaded). But what if any of those things change? And as this program scales these relationships will likely no
    #  longer be trackable. Does that mean the best option is to actually raise PreventUpdate at every callback?
    if selected_sensor is None or pose_type is None or raw_data is None:
        raise PreventUpdate

    # TODO(Jack): Technically this is not nice. To access a key (frames) without checking that it exists. However this
    # is so fundamental to the data structure I think I can be forgiven for this. Remove this todo later if it turns out
    # to be a nothing burger.
    frames = raw_data[selected_sensor]['frames']
    if frames is None:
        return {
            "rotation": {},
            "translation": {},
        }

    timestamps_ns, poses = extract_timestamps_and_poses_sorted(frames, pose_type)

    rotations = [d[:3] for d in poses]
    rot_fig = plot_pose_figure(timestamps_ns, rotations, 'Orientation', 'Axis Angle (rad)', x_name='rx', y_name='ry',
                               z_name='rz')

    translations = [d[3:] for d in poses]
    trans_fig = plot_pose_figure(timestamps_ns, translations, 'Translation', 'Meter (m)')

    return {
        "rotation": rot_fig,
        "translation": trans_fig,
    }


# ERROR IT WILL NOT UPDATE WHEN WE SELECT FROM OPTIMIZED/INITIAL
app.clientside_callback(
    """
    function(frame_idx, sensor, fig_store, processed_data, rot_fig, trans_fig) {
        // DOCUMENT
        // DOCUMENT
        // DOCUMENT
        // DOCUMENT
        // DOCUMENT
        const ctx = dash_clientside.callback_context;
        const triggered = dash_clientside.callback_context.triggered.map(t => t.prop_id);
        if (triggered.includes("translation-figure-store.data")) {
            if (fig_store && fig_store.rotation && fig_store.translation) {
                console.log("Store updated → refreshing figures");
                return [fig_store.rotation, fig_store.translation];
            }
        }
    
        // DOCUMENT
        // DOCUMENT
        // DOCUMENT
        // DOCUMENT
        // DOCUMENT
        if (!rot_fig || !trans_fig) {
            if (fig_store && fig_store.rotation && fig_store.translation) {
                return [fig_store.rotation, fig_store.translation];
            }
    
            const EMPTY_FIG = {
                data: [],
                layout: {
                    xaxis: { visible: true },
                    yaxis: { visible: true },
                    shapes: []
                }
            };
            return [EMPTY_FIG, EMPTY_FIG];
        }
    
        // ---- STEADY STATE FROM HERE ON ----
        if (!sensor || !processed_data) {
            return [rot_fig, trans_fig];
        }
    
        if (!processed_data[1] || !processed_data[1][sensor]) {
            return [rot_fig, trans_fig];
        }
    
        const timestamps = processed_data[1][sensor];
        if (!timestamps || timestamps.length <= frame_idx) {
            return [rot_fig, trans_fig];
        }
    
        const timestamp_0_ns = BigInt(timestamps[0]);
        const timestamp_i_ns = BigInt(timestamps[frame_idx]);
        const local_time_s = Number(timestamp_i_ns - timestamp_0_ns) / 1e9;
    
        // NOTE(Jack): The "paper" coordinate system goes from 0 to 1 to cover the entire figure, so we set yref to 
        // "paper" so that the y0=0 and y1=1 dimensions will draw a vertical line the entire figure height. 
        const new_shape = {
            type: 'rect',
            xref: 'x',
            yref: 'paper', 
            x0: local_time_s,
            x1: local_time_s,
            y0: 0,
            y1: 1,
            line: {color: 'black', width: 1},
        };
        
        const new_annotation = {
            x: local_time_s,
            y: 1,
            xref: 'x',
            yref: 'paper',
            text: `${frame_idx}`,
            showarrow: false,
            yanchor: 'bottom',
            xanchor: 'center',
            font: {
                color: 'white',
                size: 12
            },
            bgcolor: 'rgba(10,10,10,0.7)',
        };
        
        // WARN(Jack): This might overwrite other pre-existing shapes that we add later!
        patch = new dash_clientside.Patch;
        patch.assign(['layout', 'shapes'], [new_shape]);
        patch.assign(['layout', 'annotations'], [new_annotation]);
    
        return [patch.build(), patch.build()];
    }
    """,
    Output("rotation-graph", "figure"),
    Output("translation-graph", "figure"),
    Input("frame-id-slider", "value"),
    Input("sensor-dropdown", "value"),
    Input("translation-figure-store", "data"),
    State("processed-data-store", "data"),
    State("rotation-graph", "figure"),
    State("translation-graph", "figure"),
)


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
    # TODO(Jack): Eliminate copy and paste here in this method! We basically do the same thing twice.
    # TODO(Jack): We have now copy and pasted in several places that the marker size for xy_fig is 12 and for pixel_fig
    #  is 6. This is a hack! We need to auto scale all dimensions and all marker sizes! Or at least make them more
    #  generic.
    xy_fig = go.Figure()
    xy_fig.add_trace(
        go.Scatter(
            x=[],
            y=[],
            mode="markers",
            marker=dict(size=12),
            hovertemplate="x: %{x}<br>" + "y: %{y}<br>" + "error: %{marker.color:.3f}<extra></extra>"
        )
    )
    xy_fig.update_layout(
        title="Target Points (XY)",
        xaxis=dict(
            range=[-0.1, 0.76],  # ERROR(Jack): Do not hardcode or use global
            title=dict(text="x"),
            constrain="domain",
        ),
        yaxis=dict(
            range=[-0.1, 0.76],  # ERROR(Jack): Do not hardcode or use global
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
            hovertemplate="x: %{x}<br>" + "y: %{y}<br>" + "error: %{marker.color:.3f}<extra></extra>"
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


# TODO(Jack): Are we doing this right at all or should we be using a patch to hold the points and avoiding the json
#  stringify thing?
app.clientside_callback(
    """
    function(frame_idx, sensor, pose_type, cmax, raw_data, processed_data,  xy_fig, pixel_fig) {
        if (!sensor || !pose_type || !raw_data || !processed_data  || !xy_fig || !pixel_fig) {
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
            // If no reprojection errors are available at all then return to default marker configuration.
            xy_fig.data[0].marker = {size: 12}
            pixel_fig.data[0].marker = {size: 6}
            
            return [xy_fig, pixel_fig];
        }
        
        const reprojection_error = reprojection_errors[pose_type]
        if (!reprojection_error) {
            // If the requested reprojection error is not available then return to default marker configuration.
            xy_fig.data[0].marker = {size: 12}
            pixel_fig.data[0].marker = {size: 6}
            
            return [xy_fig, pixel_fig];
        }
        
        xy_fig.data[0].marker = {size: 12, 
                        color: reprojection_error.map(p => Math.sqrt(p[0] * p[0] + p[1] * p[1])), 
                        colorscale: "Bluered", 
                        cmin: 0, cmax: cmax,
                        showscale: true};
        pixel_fig.data[0].marker = {size: 6, 
                        color: reprojection_error.map(p => Math.sqrt(p[0] * p[0] + p[1] * p[1])), 
                        colorscale: "Bluered", 
                        cmin: 0, cmax: cmax,
                        showscale: true};
        
        return [xy_fig, pixel_fig];
    }
    """,
    Output("targets-xy-graph", "figure"),
    Output("targets-pixels-graph", "figure"),
    Input("frame-id-slider", "value"),
    Input("sensor-dropdown", "value"),
    Input('pose-type-selector', 'value'),
    Input('max-reprojection-error-input', 'value'),
    State("raw-data-store", "data"),
    State("processed-data-store", "data"),
    State("targets-xy-graph", "figure"),
    State("targets-pixels-graph", "figure"),
)

if __name__ == '__main__':
    app.run(debug=True)
