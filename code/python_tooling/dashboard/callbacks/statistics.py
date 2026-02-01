from dash import Input, Output, State, html
from dash.exceptions import PreventUpdate

from dashboard.server import app
from database.types import PoseType, SensorType


def build_sensor_statistics_div(sensor, statistics):
    # TODO(Jack): Is raise PreventUpdate the appropriate error handling strategy here?
    if sensor not in statistics:
        raise PreventUpdate

    return [
        html.Div(
            [
                html.Div(
                    "",
                    style={
                        "width": "20px",
                        "height": "20px",
                        "backgroundColor": "green" if value != 0 else "red",
                        "display": "inline-block",
                        "marginRight": "10px",
                    },
                ),
                html.Div(value, style={"width": "35px", "display": "inline-block"}),
                html.Div(key, style={"width": "200px", "display": "inline-block"}),
            ],
        )
        for key, value in statistics[sensor].items()
    ]


def register_statistics_display_update_callback(
    display_id, sensor_dropdown_id, sensor_type
):
    @app.callback(
        Output(display_id, "children"),
        Input(sensor_dropdown_id, "value"),
        State("metadata-store", "data"),
    )
    def update_sensor_statistics_display(sensor, metadata):
        # TODO(Jack): Is raise PreventUpdate the appropriate error handling strategy here?
        if sensor is None or metadata is None:
            raise PreventUpdate
        statistics, _ = metadata

        return build_sensor_statistics_div(sensor, statistics[sensor_type])


register_statistics_display_update_callback(
    "camera-statistics-display", "camera-sensor-dropdown", SensorType.Camera
)
register_statistics_display_update_callback(
    "imu-statistics-display", "imu-sensor-dropdown", SensorType.Imu
)
