from dataclasses import replace

from dash import dcc, html

from dashboard.tools.timeseries_plotting import (
    AxisConfig,
    FigureConfig,
    SubplotConfig,
    build_figure_layout,
)
from database.types import SensorType
from dashboard.tools.metadata import imu_step_selector_options

TARGET_VISUALIZATION = FigureConfig(
    "Target detections & reprojection errors",
    (
        SubplotConfig(
            "Target",
            AxisConfig("x", "m"),
            AxisConfig("y", "m"),
            1,
            ["Target Points"],
            ("blue",),
        ),
        SubplotConfig(
            "Extracted Features",
            AxisConfig("u", "pix"),
            AxisConfig("v", "pix"),
            1,
            ["Image Points"],
            ("blue",),
        ),
    ),
    "cols",
    False,
)

POSE_VISUALIZATION = FigureConfig(
    "Camera Poses",
    (
        SubplotConfig(
            "Orientation",
            AxisConfig("Time", "ns"),
            AxisConfig("Axis Angle", "rad"),
            3,
            ["rx", "ry", "rz"],
        ),
        SubplotConfig(
            "Translation",
            AxisConfig("Time", "ns"),
            AxisConfig("Position", "m"),
            3,
            ["x", "y", "z"],
        ),
    ),
    "rows",
    True,
)


def camera_layout(asset_id, label="Camera", pose_title="Camera motion"):
    return html.Section(
        [
            html.Div(
                [
                    html.H3(label),
                    html.P(
                        "Inspect target detections frame by frame. Colours show reprojection error for the selected result.",
                        className="muted",
                    ),
                ],
                className="plot-heading",
            ),
            html.Div(
                [
                    html.Button(
                        "Pause", id={"type": "pause_button", "asset_id": asset_id}
                    ),
                    html.Div(
                        [
                            html.Label("Colour scale max (px)"),
                            dcc.Input(
                                id={"type": "max_error", "asset_id": asset_id},
                                min=1e-6,
                                type="number",
                                value=1,
                            ),
                        ],
                        className="error-control",
                    ),
                    html.Div(
                        [
                            html.Span("Timestamp (ns)", className="muted"),
                            html.Code(
                                id={
                                    "type": "current_timestamp",
                                    "asset_id": asset_id,
                                    "sensor_type": SensorType.Camera,
                                },
                                children="—",
                            ),
                        ],
                        className="timestamp-display",
                    ),
                ],
                className="playback-controls",
            ),
            dcc.Slider(
                id={
                    "type": "slider",
                    "asset_id": asset_id,
                    "sensor_type": SensorType.Camera,
                },
                min=0,
                max=0,
                value=0,
                step=1,
                marks=None,
                updatemode="drag",
                tooltip={"placement": "bottom", "always_visible": False},
            ),
            dcc.Graph(
                id={"type": "extracted_targets", "asset_id": asset_id},
                figure=build_figure_layout(TARGET_VISUALIZATION),
                config={"displaylogo": False},
            ),
            dcc.Graph(
                id={
                    "type": "timeseries",
                    "asset_id": asset_id,
                    "sensor_type": SensorType.Camera,
                },
                figure=build_figure_layout(
                    replace(POSE_VISUALIZATION, title=pose_title)
                ),
                config={"displaylogo": False},
            ),
        ],
        className="plot-panel",
    )


IMU_DATA_VISUALIZATION = FigureConfig(
    "IMU measurements and residuals",
    (
        SubplotConfig(
            "Angular Velocity",
            AxisConfig("Time", "ns"),
            AxisConfig("omega", "rad/s"),
            3,
            ["omega_x", "omega_y", "omega_z"],
        ),
        SubplotConfig(
            "Linear Acceleration",
            AxisConfig("Time", "ns"),
            AxisConfig("a", "m/s^2"),
            3,
            ["acc_x", "acc_y", "acc_z"],
        ),
    ),
    "rows",
    True,
)


def imu_layout(asset_id, label="IMU", metadata=None):
    options, value = imu_step_selector_options(asset_id, metadata)
    return html.Section(
        [
            html.Div(
                [
                    html.H3(f"IMU · {label}"),
                    html.P(
                        "Measured angular velocity and acceleration. Select IMU residuals independently of the camera result above.",
                        className="muted",
                    ),
                ],
                className="plot-heading",
            ),
            html.Div(
                [
                    html.Label("IMU residual result"),
                    dcc.RadioItems(
                        id={"type": "imu-step-selector", "asset_id": asset_id},
                        options=options,
                        value=value,
                        className="result-selector",
                    ),
                    html.P("No IMU residuals available.", className="empty-state")
                    if not options else None,
                ],
                className="result-control",
            ),
            dcc.Graph(
                id={
                    "type": "timeseries",
                    "asset_id": asset_id,
                    "sensor_type": SensorType.Imu,
                },
                figure=build_figure_layout(IMU_DATA_VISUALIZATION),
                config={"displaylogo": False},
            ),
        ],
        className="plot-panel",
    )
