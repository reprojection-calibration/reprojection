from dash import html
from dash.exceptions import PreventUpdate


# TODO(Jack): Is raise PreventUpdate the appropriate error handling strategy here?
def build_sensor_statistics_div(sensor, metadata, sensor_type):
    if sensor is None or metadata is None:
        raise PreventUpdate
    statistics, _ = metadata

    statistics = statistics[sensor_type]
    if statistics is None or sensor not in statistics:
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
