from dash import dcc, html

from dashboard.tools.timeseries_plotting import FigureConfig, build_figure_layout

TARGET_VISUALIZATION = FigureConfig("", ((), ()), "cols", False)


def camera_layout(sensor_name, raw_data):
    return html.Div(
        [
            html.H3("Camera Layout"),
            dcc.Graph(
                id="targets-figure", figure=build_figure_layout(TARGET_VISUALIZATION)
            ),
        ]
    )


def imu_layout():
    return html.Div(
        [
            html.H3("IMU Layout"),
        ]
    )
