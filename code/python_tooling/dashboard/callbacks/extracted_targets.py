from dashboard.server import app

from dash import Input, Output, State, MATCH

# TODO(Jack): Do not hardcode counter ID!
app.clientside_callback(
    """
    function(composite_id, step_name, raw_data) {
        console.log("Callback triggered");
        if (!composite_id || !step_name || !raw_data) {
            return dash_clientside.no_update;
        }
        
        const sensor_name = composite_id["sensor_name"];
        console.log(sensor_name);
        
        const targets = raw_data?.[sensor_name]?.measurements?.targets?.[step_name];
        if(!targets){
            return dash_clientside.no_update;
        }
        console.log("got target");
        const key = Object.keys(targets)[0];
        const target = targets[key];
        
        const points = target.points.map(row => row.slice(0, 2));
        const pixels = target.pixels.map(row => row.slice(0, 2));
       
        const patch = new dash_clientside.Patch();
        patch.assign(['data', 0, 'x'], points.map(p => p[0]));
        patch.assign(['data', 0, 'y'], points.map(p => p[1]));
        patch.assign(['data', 1, 'x'], pixels.map(p => p[0]));
        patch.assign(['data', 1, 'y'], pixels.map(p => p[1]));
    
        return patch.build();
    }
    """,
    Output({"type": "extracted_targets", "sensor_name": MATCH}, "figure"),
    Input({"type": "extracted_targets", "sensor_name": MATCH}, "id"),
    Input("step-selector", "value"),
    State("raw-data-store", "data"),
)

