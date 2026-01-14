from dash import Input, Output, State, html
from dash.exceptions import PreventUpdate

from dashboard.server import app


@app.callback(
    Output("statistics-display", "children"),
    Input("sensor-dropdown", "value"),
    State("processed-data-store", "data"),
)
def update_statistics(selected_sensor, processed_data):
    if selected_sensor is None or processed_data is None:
        raise PreventUpdate

    statistics, _ = processed_data

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
        for key, value in statistics[selected_sensor].items()
    ]
