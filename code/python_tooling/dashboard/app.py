# NOTE(Jack): These might look unused but they need to be imported here so that the callbacks can be registered with the
# Dash instance. Please see the answer here
# https://community.plotly.com/t/splitting-callback-definitions-in-multiple-files/10583/2
import callbacks.data_loading
import callbacks.extracted_target
import callbacks.pose_graph
import callbacks.slider
import callbacks.statistics
from dash import dcc, html

from dashboard.server import app
from database.types import PoseType

# TODO(Jack): Use a css style sheet instead of individually specifying the properties everywhere! This does not scale.


app.layout = html.Div(
    [
        html.H2("Reprojection - The future is calibrated."),
        html.Div(
            children=[
                html.Div(
                    children=[
                        html.Label(
                            children="Directory",
                        ),
                        dcc.Input(
                            id="database-directory-input",
                            type="text",
                            value="../../test_data",
                            placeholder="/path/to/database/directory",
                            debounce=True,
                            persistence=True,
                            persistence_type="local",
                            style={"width": "300px"},
                        ),
                    ],
                    style={
                        "alignItems": "center",
                        "display": "flex",
                        "flexDirection": "row",
                        "gap": "10px",
                        "margin": "10px",
                        "flex": "1",
                    },
                ),
                html.Div(
                    children=[
                        html.Label(
                            children="Load",
                        ),
                        dcc.Dropdown(
                            id="database-dropdown",
                            placeholder="Select a database",
                            style={"width": "300px"},
                        ),
                        html.Button(
                            children="Refresh Database List",
                            id="refresh-database-list-button",
                            n_clicks=0,
                        ),
                    ],
                    style={
                        "alignItems": "center",
                        "display": "flex",
                        "flexDirection": "row",
                        "gap": "10px",
                        "margin": "10px",
                        "flex": "1",
                    },
                ),
                # The animation plays by default therefore the button is initialized with the pause graphic
                html.Button(
                    children="⏸ Pause",
                    id="play-button",
                    n_clicks=0,
                    style={
                        "width": "50px",
                    },
                ),
            ],
            style={
                "alignItems": "flex-start",
                "display": "flex",
                "flexDirection": "row",
                "gap": "10px",
                "margin": "10px",
                "flex": "1",
            },
        ),
        dcc.Tabs(
            [
                dcc.Tab(
                    children=[
                        html.Div(
                            children=[
                                html.Div(
                                    children=[
                                        html.Div(
                                            children=[
                                                html.Label(
                                                    children="Select",
                                                ),
                                                dcc.Dropdown(
                                                    id="camera-sensor-dropdown",
                                                    placeholder="Select a camera sensor",
                                                    style={"width": "300px"},
                                                ),
                                            ],
                                            style={
                                                "alignItems": "center",
                                                "display": "flex",
                                                "flexDirection": "row",
                                                "gap": "10px",
                                                "margin": "10px",
                                                "flex": "1",
                                            },
                                        ),
                                        html.Div(
                                            children=[
                                                html.Div(
                                                    [
                                                        html.Div(
                                                            id="camera-statistics-display"
                                                        ),
                                                    ]
                                                ),
                                            ],
                                            style={
                                                "alignItems": "top",
                                                "display": "flex",
                                                "flexDirection": "row",
                                                "gap": "10px",
                                                "margin": "10px",
                                                "flex": "1",
                                            },
                                        ),
                                        html.Div(
                                            children=[
                                                html.Label(
                                                    children="Pose Type",
                                                ),
                                                dcc.RadioItems(
                                                    id="pose-type-selector",
                                                    options=[
                                                        {
                                                            "label": "Initial",
                                                            "value": PoseType.Initial,
                                                        },
                                                        {
                                                            "label": "Optimized",
                                                            "value": PoseType.Optimized,
                                                        },
                                                    ],
                                                    value=PoseType.Initial,
                                                ),
                                            ],
                                            style={
                                                "alignItems": "top",
                                                "display": "flex",
                                                "flexDirection": "column",
                                                "gap": "10px",
                                                "margin": "10px",
                                                "flex": "1",
                                            },
                                        ),
                                    ],
                                    style={
                                        "alignItems": "top",
                                        "display": "flex",
                                        "flexDirection": "row",
                                        "gap": "10px",
                                        "margin": "10px",
                                    },
                                ),
                                html.Div(
                                    children=[
                                        html.Div(
                                            children=[
                                                dcc.Slider(
                                                    id="camera-frame-id-slider",
                                                    marks=None,
                                                    min=0,
                                                    max=0,
                                                    step=1,
                                                    value=0,
                                                    tooltip={
                                                        "placement": "top",
                                                        "always_visible": True,
                                                    },
                                                    updatemode="drag",
                                                ),
                                            ],
                                            style={
                                                "width": "80%",
                                            },
                                        ),
                                        html.Div(
                                            children=[
                                                html.P("Current timestamp (ns)"),
                                                html.Div(
                                                    id="camera-timestamp-display",
                                                ),
                                            ],
                                        ),
                                    ],
                                    style={
                                        "alignItems": "top",
                                        "display": "flex",
                                        "flexDirection": "row",
                                        "gap": "10px",
                                        "margin": "10px",
                                        "flex": "1",
                                    },
                                ),
                                html.Div(
                                    children=[
                                        html.Label(
                                            children="Max reprojection error (pix)",
                                        ),
                                        dcc.Input(
                                            id="max-reprojection-error-input",
                                            type="number",
                                            min=0,
                                            max=1000,
                                            value=1,
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
                                html.Div(
                                    children=[
                                        dcc.Graph(
                                            id="camera-orientation-graph",
                                        ),
                                        dcc.Graph(
                                            id="camera-translation-graph",
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
                            style={
                                "display": "flex",
                                "flexDirection": "column",
                                "gap": "20px",
                                "flex": "1",
                                "width": "100%",
                            },
                        ),
                    ],
                    label="Camera Calibration",
                ),
                # TODO(Jack): Almost the entire section of both tabs is copy and pasted!
                dcc.Tab(
                    children=[
                        html.Div(
                            children=[
                                html.Div(
                                    children=[
                                        html.Div(
                                            children=[
                                                html.Label(
                                                    children="Select",
                                                ),
                                                dcc.Dropdown(
                                                    id="imu-sensor-dropdown",
                                                    placeholder="Select a imu sensor",
                                                    style={"width": "300px"},
                                                ),
                                            ],
                                            style={
                                                "alignItems": "center",
                                                "display": "flex",
                                                "flexDirection": "row",
                                                "gap": "10px",
                                                "margin": "10px",
                                                "flex": "1",
                                            },
                                        ),
                                        html.Div(
                                            children=[
                                                html.Div(
                                                    [
                                                        html.Div(
                                                            id="imu-statistics-display"
                                                        ),
                                                    ]
                                                ),
                                            ],
                                            style={
                                                "alignItems": "top",
                                                "display": "flex",
                                                "flexDirection": "row",
                                                "gap": "10px",
                                                "margin": "10px",
                                                "flex": "1",
                                            },
                                        ),
                                    ],
                                    style={
                                        "alignItems": "top",
                                        "display": "flex",
                                        "flexDirection": "row",
                                        "gap": "10px",
                                        "margin": "10px",
                                    },
                                ),
                                html.Div(
                                    children=[
                                        html.Div(
                                            children=[
                                                dcc.Slider(
                                                    id="imu-frame-id-slider",
                                                    marks=None,
                                                    min=0,
                                                    max=0,
                                                    step=1,
                                                    value=0,
                                                    tooltip={
                                                        "placement": "top",
                                                        "always_visible": True,
                                                    },
                                                    updatemode="drag",
                                                ),
                                            ],
                                            style={
                                                "width": "80%",
                                            },
                                        ),
                                        html.Div(
                                            children=[
                                                html.P("Current timestamp (ns)"),
                                                html.Div(
                                                    id="imu-timestamp-display",
                                                ),
                                            ],
                                        ),
                                    ],
                                    style={
                                        "alignItems": "top",
                                        "display": "flex",
                                        "flexDirection": "row",
                                        "gap": "10px",
                                        "margin": "10px",
                                        "flex": "1",
                                    },
                                ),
                                html.Div(
                                    children=[
                                        dcc.Graph(
                                            id="imu-angular-velocity-graph",
                                        ),
                                        dcc.Graph(
                                            id="imu-linear-acceleration-graph",
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
                            style={
                                "display": "flex",
                                "flexDirection": "column",
                                "gap": "20px",
                                "flex": "1",
                                "width": "100%",
                            },
                        ),
                    ],
                    label="Imu Calibration",
                ),
            ]
        ),
        # Components without a visual representation are found here at the bottom (ex. Interval, Store etc.)
        dcc.Interval(
            disabled=False,
            id="play-interval",
            interval=50,
        ),
        # NOTE(Jack): What we want to prevent is that big chunks of data get sent to and from the browse more than they
        # need to. As the calibration data might be 10, 30, or even 100mb it is important to make sure we only send that to
        # the browser when we need to. Therefore, we designed these two data stores, one heavy one (raw-camera-data-store) and one
        # light one (processed-data-store). In the light one we should find all the metadata required to parameterize and
        # build most of the dashboard (ex. timestamps, number of frames etc.) and the heavy one we find the entire dataset
        # which we actually need to process to build our figures. Unless you absolutely need the raw data you should only
        # use the processed data!
        dcc.Store(id="raw-camera-data-store"),
        dcc.Store(id="raw-imu-data-store"),
        dcc.Store(id="processed-data-store"),
    ]
)
