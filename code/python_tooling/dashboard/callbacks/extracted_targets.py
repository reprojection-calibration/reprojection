from dash import MATCH, Input, Output, State

from dashboard.server import app

# TODO(Jack): Do not hardcode counter ID!
app.clientside_callback(
    """
    function(composite_id, step_name, raw_data) {
        if (!composite_id || !step_name || !raw_data) {
            return dash_clientside.no_update;
        }
        
        const sensor_name = composite_id["sensor_name"];
        const targets = raw_data?.[sensor_name]?.measurements?.targets;
        if(!targets){
            return dash_clientside.no_update;
        }

        const key = Object.keys(targets)[0];
        const target = targets[key];
        
        const points = target.points.map(row => row.slice(0, 2));
        const pixels = target.pixels.map(row => row.slice(0, 2));
       
        const patch = new dash_clientside.Patch();
        patch.assign(['data', 0, 'x'], points.map(p => p[0]));
        patch.assign(['data', 0, 'y'], points.map(p => p[1]));
        patch.assign(['data', 1, 'x'], pixels.map(p => p[0]));
        patch.assign(['data', 1, 'y'], pixels.map(p => p[1]));
        
        const reprojection_errors = raw_data?.[sensor_name]?.reprojection_error?.[step_name];
        if(reprojection_errors){
            const reprojection_error = reprojection_errors[key]

            patch.assign(['data', 0, 'marker'], {
                size: 12,
                color: reprojection_error.map(p => Math.sqrt(p[0] * p[0] + p[1] * p[1])),
                colorscale: "Bluered",
                cmin: 0,
                cmax: 1,
            });
            patch.assign(['data', 1, 'marker'], {
                size: 12,
                color: reprojection_error.map(p => Math.sqrt(p[0] * p[0] + p[1] * p[1])),
                colorscale: "Bluered",
                cmin: 0,
                cmax: 1,
            });
            
            patch.assign(['data', 0, 'hovertemplate'], "x: %{x}<br>y: %{y}<br>error: %{marker.color:.3f}<extra></extra>");
            patch.assign(['data', 1, 'hovertemplate'], "x: %{x}<br>y: %{y}<br>error: %{marker.color:.3f}<extra></extra>");
        }
    
        return patch.build();
    }
    """,
    Output({"type": "extracted_targets", "sensor_name": MATCH}, "figure"),
    Input({"type": "extracted_targets", "sensor_name": MATCH}, "id"),
    Input("step-selector", "value"),
    State("raw-data-store", "data"),
)
