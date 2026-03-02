from dash import dcc, html

from dashboard.tools.timeseries_plotting import (
    AxisConfig,
    FigureConfig,
    SubplotConfig,
    build_figure_layout,
)

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
            dcc.Graph(
                id={
                    "type": "targets",
                    "sensor": sensor_name,
                },
                figure=build_figure_layout(TARGET_VISUALIZATION),
            ),
            dcc.Graph(
                id={"type": "pose", "sensor": sensor_name},
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


def imu_layout():
    return html.Div(
        [
            html.H3("IMU Layout"),
            dcc.Graph(
                id="imu-data-figure", figure=build_figure_layout(IMU_DATA_VISUALIZATION)
            ),
        ]
    )
