from dash import dcc, html

from dashboard.tools.timeseries_plotting import (
    AxisConfig,
    FigureConfig,
    SubplotConfig,
    build_figure_layout,
)
from database.types import SensorType

TARGET_VISUALIZATION = FigureConfig(
    "Feature Extraction",
    (
        SubplotConfig("Target", AxisConfig("x", "m"), AxisConfig("y", "m"), 1),
        SubplotConfig(
            "Extracted Features", AxisConfig("u", "pix"), AxisConfig("v", "pix"), 1
        ),
    ),
    "cols",
    False,
)

POSE_VISUALIZATION = FigureConfig(
    "Camera Poses",
    (
        SubplotConfig(
            "Orientation", AxisConfig("Time", "s"), AxisConfig("Axis Angle", "rad"), 3
        ),
        SubplotConfig(
            "Translation", AxisConfig("Time", "s"), AxisConfig("Position", "m"), 3
        ),
    ),
    "rows",
    True,
)


# NOTE(Jack): We use pattern matching callbacks to facilitate the dynamic layout -
#       https://dash.plotly.com/pattern-matching-callbacks
def camera_layout(sensor_name):
    return html.Div(
        [
            html.H3("Camera Layout"),
            html.Div(
                id={
                    "type": "current_timestamp",
                    "sensor_name": sensor_name,
                    "sensor_type": SensorType.Camera,
                },
                children="N/A",
            ),
            dcc.Input(
                id={
                    "type": "max_error",
                    "sensor_name": sensor_name,
                },
                min=1e-6,
                type="number",
                value=1,
                style={
                    "width": "10%",
                    "minWidth": "60px",
                },
            ),
            dcc.Graph(
                id={
                    "type": "extracted_targets",
                    "sensor_name": sensor_name,
                },
                figure=build_figure_layout(TARGET_VISUALIZATION),
            ),
            html.Div(
                [
                    html.Div(
                        html.Button(
                            "Pause",
                            id={"type": "pause_button", "sensor_name": sensor_name},
                            # TODO(Jack): Style largely copy and pasted from the metadata cards. Centralize!
                            style={
                                "backgroundColor": "white",
                                "border": "1px solid #ddd",
                                "borderRadius": "6px",
                                "boxShadow": "0px 1px 2px rgba(0,0,0,0.05)",
                                "width": "90%",
                            },
                        ),
                        style={
                            "width": "10%",
                            "minWidth": "60px",
                        },
                    ),
                    html.Div(
                        dcc.Slider(
                            id={
                                "type": "slider",
                                "sensor_name": sensor_name,
                                "sensor_type": SensorType.Camera,
                            },
                            min=0,
                            value=0,
                            step=1,
                            marks=None,
                            updatemode="drag",
                            tooltip={
                                "placement": "top",
                                "always_visible": True,
                            },
                        ),
                        style={
                            "width": "90%",
                        },
                    ),
                ],
                style={
                    "display": "flex",
                    "alignItems": "center",
                    "width": "100%",  # full row width
                },
            ),
            dcc.Graph(
                id={
                    "type": "timeseries",
                    "sensor_name": sensor_name,
                    "sensor_type": SensorType.Camera,
                },
                figure=build_figure_layout(POSE_VISUALIZATION),
            ),
        ]
    )


IMU_DATA_VISUALIZATION = FigureConfig(
    "Imu Data",
    (
        SubplotConfig(
            "Angular Velocity", AxisConfig("Time", "s"), AxisConfig("omega", "rad/s"), 3
        ),
        SubplotConfig(
            "Linear Acceleration", AxisConfig("Time", "s"), AxisConfig("a", "m/s^2"), 3
        ),
    ),
    "rows",
    True,
)


def imu_layout(sensor_name):
    return html.Div(
        [
            html.H3("IMU Layout"),
            dcc.Graph(
                id={
                    "type": "timeseries",
                    "sensor_name": sensor_name,
                    "sensor_type": SensorType.Imu,
                },
                figure=build_figure_layout(IMU_DATA_VISUALIZATION),
            ),
        ]
    )
