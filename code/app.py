import os
import sqlite3
from dash import Dash, dcc, html, Input, Output
import plotly.express as px

# ------------------------
# Configuration
# ------------------------
DB_DIR = "./test_data"  # directory containing .db3 files
TIMESTAMP_COL = "timestamp"
VALUE_COL = "value"


def list_databases():
    """Return list of .db3 files in DB_DIR"""
    if not os.path.exists(DB_DIR):
        return []
    return sorted([f for f in os.listdir(DB_DIR) if f.endswith(".db3")])


# ------------------------
# Dash App
# ------------------------
app = Dash(__name__)

app.layout = html.Div(
    style={"width": "80%", "margin": "auto"},
    children=[
        html.H2("Calibration Database Viewer"),

        html.Div([
            dcc.Dropdown(
                id="db-dropdown",
                options=[{"label": db, "value": db} for db in list_databases()],
                placeholder="Select a database",
                style={"width": "70%"}
            ),
            html.Button("Reload Databases", id="reload-btn", n_clicks=0)
        ], style={"display": "flex", "gap": "10px"}),

        dcc.Graph(id="timeseries-plot")
    ]
)


# ------------------------
# Callbacks
# ------------------------
@app.callback(
    Output("db-dropdown", "options"),
    Input("reload-btn", "n_clicks")
)
def reload_database_list(_):
    return [{"label": db, "value": db} for db in list_databases()]


@app.callback(
    Output("timeseries-plot", "figure"),
    Input("db-dropdown", "value")
)
def update_plot(selected_db):
    if not selected_db:
        return px.line(title="No database selected")

    # df = load_database(selected_db)

    fig = px.line(
        df,
        x=TIMESTAMP_COL,
        y=VALUE_COL,
        title=f"Timestamp Series: {selected_db}"
    )
    fig.update_layout(xaxis_title="Timestamp", yaxis_title="Value")

    return fig


if __name__ == "__main__":
    app.run(debug=True)
