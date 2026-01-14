from dash import dcc, html, Input, Output, State
import plotly.graph_objects as go



from server import app

from database.types import PoseType

import data_loading_callbacks
import slider_callbacks
import callbacks_statistics
import callbacks_pose_graph

# TODO(Jack): Use a css style sheet instead of individually specifying the properties everywhere! This does not scale.

# TODO(Jack): Place meta data like this in config file or in database. For now we use globals...
IMAGE_DIMENSIONS = (512, 512)

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
    dcc.Store(id='pose-figure-store'),
])




# TODO(Jack): Technically we only need the sensor name to get the frame-id-slider output, but this does not necessarily
#  have anything to do with configuring the figures initially. It might make sense to move the frame id slider
#  dependency to another place that is more related or independent than here.
@app.callback(
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
