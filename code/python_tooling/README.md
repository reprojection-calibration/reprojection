# Python tooling data model

The dashboard and reports use the tables in `../resources/sql` directly.
`load_calibration_database()` returns a dictionary of pandas DataFrames with
ordinary SQL columns, including all identifiers. Protobuf `data` columns are
decoded into arrays/lists; timestamps remain integer nanoseconds. There is no
sensor-name or step-type keyed timeseries representation.

`parse_workflows()` reads workflow membership into a `Workflow`, whose assets
and steps are indexed by their database IDs. `process_workflow()` selects the
artifact tables belonging to those steps and assets. Extrinsics retain both
`asset_a_id` and `asset_b_id`. `select_rows()` filters tables without changing
their columns or identities.

Use `(step_id, asset_id, timestamp_ns)` to identify a sample. Poses and errors
refer to observations through `(source_step_id, asset_id, timestamp_ns)`;
joining on the timestamp alone can mix cameras or processing steps. Target
metadata belongs to a target asset. The schema does not associate each
extracted-target row with a target asset, so the dashboard derives target plot
bounds from the extracted points.

At the Dash boundary, `serialize_workflow()` transports the same tables as
JSON row records. Only `timestamp_ns` changes type, to a decimal string, because
JavaScript numbers cannot represent every nanosecond timestamp. Sensor and
step selectors use database IDs; names and step types are display labels.
`selected-targets-store` holds a selection of extracted-target rows for the
current camera and the selected result's source steps. Metadata counts rows
by table, step, and asset, independently of the plotted quantities.

Camera poses remain camera-from-world in the tables. The timeseries plot
inverts them to display camera motion in world coordinates. IMU error bars and
report reprojection plots join through source identities. Measurement intervals
are computed separately for each step and asset. TOML extrinsics retain every
pair and identify the producing step with `step_id`.

Run the tests from the repository root:

```sh
REPROJECTION_SQL_PYTHON_DIR="$PWD/code/resources/sql" \
  python -m unittest discover -s code/python_tooling/tests -t code/python_tooling
```

Use the project's Python environment. The clientside callback test also uses
Node.js when available.

## Testing in CLion

Create a **Python tests** run configuration targeting
`<repository>/code/python_tooling/tests`, select the project's Python environment,
and set `REPROJECTION_SQL_PYTHON_DIR=<repository>/code/resources/sql`.
