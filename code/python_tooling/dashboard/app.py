from dash import Dash, dcc, html, Input, Output, callback
import os
from database.load_image_frame_data import load_image_frame_data

DB_DIR = '../../test_data/'

app = Dash()
app.layout = html.Div([
    html.H2("Reprojection - the future is calibrated."),

    html.Div(
        style={"display": "flex", "gap": "10px", "marginBottom": "20px"},
        children=[
            html.Label('Load'),
            dcc.Dropdown(id='database-dropdown', placeholder="Select a database", style={"width": "50%"}),
            html.Button('Refresh Database List', id='refresh-database-list-button', n_clicks=0),
        ]
    ),

    html.Div(
        style={"display": "flex", "gap": "10px", "marginBottom": "20px"},
        children=[
            html.Label('Select'),
            dcc.Dropdown(id='sensor-dropdown', placeholder="Select a camera sensor", style={"width": "50%"}),
        ]
    ),

    dcc.Store(id='database-store'),
])


@callback(
    Output('database-dropdown', 'options'),
    Input('refresh-database-list-button', 'n_clicks')
)
def refresh_database_list(_):
    if not os.path.exists(DB_DIR):
        return []

    return sorted([f for f in os.listdir(DB_DIR) if f.endswith(".db3")])


@callback(
    Output('database-store', 'data'),
    Input('database-dropdown', 'value')
)
def load_database_to_store(db_file):
    if not db_file:
        return None

    full_path = DB_DIR + db_file
    if not os.path.isfile(full_path):
        return None

    return load_image_frame_data(full_path)


@callback(
    Output('sensor-dropdown', 'options'),
    Input('database-store', 'data')
)
def refresh_sensor_list(data):
    if not data:
        return []

    return list(data.keys())


if __name__ == '__main__':
    app.run(debug=True)
