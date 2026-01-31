from dash import Input, Output, State, html
from dash.exceptions import PreventUpdate

from dashboard.server import app
from database.types import PoseType, SensorType

# TODO(Jack): Is there any way to avoid copying this completely for the IMU case?


def register_statistics_display_update_callback(
    display_id, sensor_dropdown_id, sensor_type
):
    @app.callback(
        Output(display_id, "children"),
        Input(sensor_dropdown_id, "value"),
        State("processed-data-store", "data"),
    )
    def update_sensor_statistics_display(sensor, processed_data):
        # TODO(Jack): Do not raise PreventUpdate! That is too extreme of an error handling strategy, just do a no update.
        if sensor is None or processed_data is None:
            raise PreventUpdate

        statistics, _ = processed_data
        sensor_statistics = statistics[sensor_type]

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
            for key, value in sensor_statistics[sensor].items()
        ]


register_statistics_display_update_callback(
    "camera-statistics-display", "camera-sensor-dropdown", SensorType.Camera
)
register_statistics_display_update_callback(
    "imu-statistics-display", "imu-sensor-dropdown", SensorType.Imu
)
