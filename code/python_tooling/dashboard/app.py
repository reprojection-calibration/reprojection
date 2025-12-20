from dash import Dash, dcc, html, Input, Output, State, callback
import os
import plotly.graph_objects as go
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

    dcc.Graph(id='rotation-graph'),
    dcc.Graph(id='translation-graph'),

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


@callback(
    Output('rotation-graph', 'figure'),
    Output('translation-graph', 'figure'),
    Input('sensor-dropdown', 'value'),
    State('database-store', 'data'),
)
def update_translation_graph(selected_sensor, data):
    if not selected_sensor or not data:
        return {}, {}

    times = data[selected_sensor]['_times']

    frames = data[selected_sensor]['data']
    frames = dict(sorted(frames.items()))
    external_poses = [frames[key]['external_pose'] for key in frames]

    rotations = [x[1:4] for x in external_poses]  # rx, ry, rz
    # TODO(Jack): Use numpy arrays here if possible?
    rotations_x = [r[0] for r in rotations]
    rotations_y = [r[1] for r in rotations]
    rotations_z = [r[2] for r in rotations]

    # TODO(Jack): Make plot legend consistent
    rot_fig = go.Figure()
    rot_fig.add_scatter(x=times, y=rotations_x, mode='lines+markers', name='X')
    rot_fig.add_scatter(x=times, y=rotations_y, mode='lines+markers', name='Y')
    rot_fig.add_scatter(x=times, y=rotations_z, mode='lines+markers', name='Z')

    rot_fig.update_layout(
        xaxis_title='Time(ns)',
        yaxis_title='Axis Angle Rotation'
    )

    trans_fig = go.Figure()

    return rot_fig, trans_fig


if __name__ == '__main__':
    app.run(debug=True)
