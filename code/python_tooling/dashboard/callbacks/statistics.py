from dash import Input, Output, State, html
from dash.exceptions import PreventUpdate

from dashboard.server import app
from database.types import PoseType

from database.types import SensorType


# TODO(Jack): Is there any way to avoid copying this completely for the IMU case?
@app.callback(
    Output("camera-statistics-display", "children"),
    Input("camera-sensor-dropdown", "value"),
    State("processed-data-store", "data"),
)
def update_camera_statistics(selected_camera_sensor, processed_data):
    # TODO(Jack): Do not raise PreventUpdate! That is too extreme of an error handling strategy, just do a no update.
    if selected_camera_sensor is None or processed_data is None:
        raise PreventUpdate

    statistics, _ = processed_data
    camera_statistics = statistics[
        SensorType.Camera]  # TODO WE NEED TO BE MORE PROTECTIVE HERE! What if there are no cameras?

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
        for key, value in camera_statistics[selected_camera_sensor].items()
    ]
