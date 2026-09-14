from dash import dcc, html

from dashboard.server import app
from dashboard.tools.workflow import stage_navigation

# Importing these modules registers their callbacks with the shared Dash instance.
from .callbacks import (
    data_loading,
    extracted_targets,
    metadata,
    populate_sensor_panel,
    slider,
    timeseries_6d_figures,
    workflow,
)

app.layout = html.Div(
    [
        html.Header(
            [
                html.Div(
                    [
                        html.Span("REPROJECTION", className="eyebrow"),
                        html.H1("Calibration workspace"),
                    ]
                ),
                html.P(
                    "Individual cameras → Stereo rig → Visual-inertial",
                    className="workflow-path",
                ),
            ],
            className="app-header",
        ),
        html.Section(
            [
                html.Div(
                    [
                        html.Label(
                            "Database directory", htmlFor="database-directory-input"
                        ),
                        html.Div(
                            [
                                dcc.Input(
                                    id="database-directory-input",
                                    type="text",
                                    debounce=True,
                                    value="/workspace/",
                                    persistence=True,
                                    persistence_type="local",
                                ),
                                html.Button(
                                    "Refresh",
                                    id="refresh-database-list-button",
                                    n_clicks=0,
                                ),
                            ],
                            className="directory-control",
                        ),
                    ],
                    className="database-field",
                ),
                html.Div(
                    [
                        html.Label("Database", htmlFor="database-selection-dropdown"),
                        dcc.Dropdown(
                            id="database-selection-dropdown",
                            options=[],
                            value=None,
                            placeholder="Choose database…",
                            clearable=False,
                        ),
                    ],
                    className="database-field",
                ),
                html.Div(
                    [
                        html.Label("Workflow", htmlFor="workflow-selection-dropdown"),
                        dcc.Dropdown(
                            id="workflow-selection-dropdown",
                            options=[],
                            value=None,
                            placeholder="Choose workflow…",
                            clearable=False,
                        ),
                    ],
                    className="database-field",
                ),
            ],
            className="database-controls",
        ),
        html.Nav(
            [
                dcc.RadioItems(
                    id="stage-selector",
                    options=stage_navigation(None)[0],
                    value=None,
                    className="stage-navigation",
                    labelClassName="stage-choice",
                ),
            ],
            **{"aria-label": "Calibration stages"}
        ),
        html.Main(
            [
                html.Section(id="stage-overview", className="stage-overview"),
                html.Section(
                    [
                        html.Div(
                            [
                                html.Label(
                                    "Inspect camera",
                                    htmlFor="sensor-selection-dropdown",
                                ),
                                dcc.Dropdown(
                                    id="sensor-selection-dropdown",
                                    options=[],
                                    value=None,
                                    placeholder="Choose camera…",
                                    clearable=False,
                                ),
                            ],
                            className="camera-control",
                        ),
                        html.Div(
                            [
                                html.Label("Compare camera results"),
                                dcc.RadioItems(
                                    id="step-selector",
                                    options=[],
                                    value=None,
                                    className="result-selector",
                                ),
                            ],
                            className="result-control",
                        ),
                    ],
                    className="result-controls",
                ),
                html.Section(id="result-summary", className="result-summary"),
                html.Div(id="sensor-content-container"),
                html.Details(
                    [
                        html.Summary("Camera and target details"),
                        html.Div(id="sensor-statistics-container"),
                    ],
                    className="data-details",
                ),
            ]
        ),
        dcc.Interval(disabled=False, id="play-interval", interval=50),
        dcc.Store(id="workflow-data-store"),
        dcc.Store(id="selected-targets-store"),
        dcc.Store(id="metadata-store"),
    ],
    className="calibration-app",
)
