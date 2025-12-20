from dash import Dash, dcc, html, Input, Output, callback
import os

DB_DIR = '../../test_data'

app = Dash()
app.layout = html.Div([
    html.Button('Refresh Database List', id='refresh-database-list-button', n_clicks=0),
    dcc.Dropdown(id='database-dropdown'),
    html.Div(id='dd-output-container')  # REMOVE
])


@callback(
    Output('database-dropdown', 'options'),
    Input('refresh-database-list-button', 'n_clicks')
)
def refresh_database_list(value):
    if not os.path.exists(DB_DIR):
        return []

    return sorted([f for f in os.listdir(DB_DIR) if f.endswith(".db3")])


if __name__ == '__main__':
    app.run(debug=True)
