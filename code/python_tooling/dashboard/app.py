# NOTE(Jack): These might look unused but they need to be imported here so that the callbacks can be registered with the
# Dash instance. Please see the answer here
# https://community.plotly.com/t/splitting-callback-definitions-in-multiple-files/10583/2
import callbacks.data_loading
from dash import dcc, html

from dashboard.server import app

# TODO(Jack): Use a css style sheet instead of individually specifying the properties everywhere! This does not scale.


app.layout = html.Div(
    [
        html.Div(
            [
                html.Div(
                    [
                        html.Label("Database Directory"),
                        dcc.Input(
                            id="database-directory-input",
                            type="text",
                            placeholder="Enter path to database directory...",
                            debounce=True,
                            style={"width": "100%"},
                            value="../../code/test_data/",
                            persistence=True,
                            persistence_type="local",
                        ),
                    ],
                    style={"flex": "3", "minWidth": "250px"},
                ),
                html.Div(
                    [
                        html.Label(" "),
                        html.Button(
                            "Refresh",
                            id="refresh-database-list-button",
                            n_clicks=0,
                            style={"width": "100%"},
                        ),
                    ],
                    style={"flex": "1", "minWidth": "120px"},
                ),
                html.Div(
                    [
                        html.Label("Select Database"),
                        dcc.Dropdown(
                            id="database-selection-dropdown",
                            options=[],
                            value=None,
                            placeholder="Choose database...",
                            clearable=False,
                        ),
                    ],
                    style={"flex": "2", "minWidth": "200px"},
                ),
                html.Div(
                    [
                        html.Label("Select Sensor"),
                        dcc.Dropdown(
                            id="sensor-selection-dropdown",
                            options=[],
                            value=None,
                            placeholder="Choose sensor...",
                            clearable=False,
                        ),
                    ],
                    style={"flex": "2", "minWidth": "200px"},
                ),
                html.Div(
                    id="sensor-statistics-container",
                    style={
                        "marginTop": "15px",
                        "paddingTop": "10px",
                        "borderTop": "1px solid #ddd",
                        "display": "flex",
                        "flexWrap": "wrap",
                        "gap": "20px",
                    },
                ),
            ],
            style={
                "display": "flex",
                "gap": "15px",
                "alignItems": "flex-end",
                "padding": "15px",
                "borderBottom": "1px solid #ddd",
                "backgroundColor": "#f9f9f9",
            },
        ),
        # NOTE(Jack): Components without a visual representation are found here at the bottom (ex. Interval, Store etc.)
        dcc.Interval(
            disabled=False,
            id="play-interval",
            interval=50,
        ),
        # NOTE(Jack): What we want to prevent is that big chunks of data get sent to and from the browse more than they
        # need to. As the calibration data might be 10, 30, or even 100mb it is important to make sure we only send to
        # the browser when we need to. Unless you absolutely need the raw data you should not use it!
        dcc.Store(id="raw-data-store"),
        dcc.Store(id="metadata-store"),
        dcc.Store(id="timestamp-store"),
    ]
)
