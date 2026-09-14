import ast
import json
import shutil
import subprocess
import unittest
from pathlib import Path

import dashboard.callbacks.extracted_targets
import dashboard.callbacks.slider


def clientside_function(module):
    tree = ast.parse(Path(module.__file__).read_text())
    call = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "clientside_callback"
    )
    return ast.literal_eval(call.args[0])


@unittest.skipUnless(
    shutil.which("node"), "Node.js is needed to exercise clientside callbacks"
)
class TestClientsideCallbacks(unittest.TestCase):
    def test_extracted_targets_match_exact_source_and_timestamp(self):
        target = dict(
            step_id=20,
            asset_id=1,
            timestamp_ns="1700000000000000001",
            data=dict(points=[[0, 0, 0]], pixels=[[50, 60]], indices=[[1, 2]]),
        )
        error = dict(
            step_id=30,
            source_step_id=20,
            asset_id=1,
            sample_timestamp_ns=target["timestamp_ns"],
            data=[[3, 4]],
        )
        other_errors = [
            dict(error, source_step_id=21, data=[[90, 0]]),
            dict(error, sample_timestamp_ns="1700000000000000002", data=[[80, 0]]),
            dict(error, asset_id=2, data=[[70, 0]]),
            dict(error, step_id=31, data=[[60, 0]]),
        ]
        payload = dict(
            tables=dict(
                reprojection_errors=other_errors + [error],
                camera_info=[dict(asset_id=1, width=640, height=480)],
            )
        )
        script = """
const assert = require('node:assert/strict');
global.dash_clientside = {Patch: class {
    constructor() { this.values = {}; }
    assign(path, value) { this.values[path.join('.')] = value; }
    build() { return this.values; }
}};
"""
        script += (
            "const render = ("
            + clientside_function(dashboard.callbacks.extracted_targets)
            + ");\n"
        )
        script += (
            "const timestamp = ("
            + clientside_function(dashboard.callbacks.slider)
            + ");\n"
        )
        script += "const targets = " + json.dumps([target]) + ";\n"
        script += "const data = " + json.dumps(payload) + ";\n"
        script += """
const patch = render(0, targets, 30, data, 10, {asset_id: 1});
assert.deepEqual(patch['data.0.marker'].color, [5]);
assert.deepEqual(patch['layout.xaxis2.range'], [0, 640]);
assert.equal(timestamp(0, targets), '1700000000000000001');
assert.equal(timestamp(1, targets), '');
assert.deepEqual(render(1, targets, 30, data, 10, {asset_id: 1})['data.0.x'], []);
assert.equal(render(0, targets, 99, data, 10, {asset_id: 1})['data.0.marker'].color, 'darkgray');
assert.deepEqual(render(0, targets, 30, data, 10, {asset_id: 2})['data.0.x'], []);
"""
        subprocess.run(
            [shutil.which("node"), "-e", script],
            check=True,
            capture_output=True,
            text=True,
        )
