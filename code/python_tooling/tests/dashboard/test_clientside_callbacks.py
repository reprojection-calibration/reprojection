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
            "const renderAll = ("
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
const render = (index, targets, step, data, max, id) =>
    renderAll([index], targets, step, data, [max], 'single_cam', id);
const patch = render(0, targets, 30, data, 10, {asset_id: 1});
assert.deepEqual(patch['data.0.marker'].color, [5]);
assert.deepEqual(patch['layout.xaxis2.range'], [0, 640]);
assert.equal(timestamp(0, targets), '1700000000000000001');
assert.equal(timestamp(1, targets), '');
assert.deepEqual(render(1, targets, 30, data, 10, {asset_id: 1})['data.0.x'], []);
assert.equal(render(0, targets, 99, data, 10, {asset_id: 1})['data.0.marker'].color, 'darkgray');
assert.deepEqual(render(0, targets, 30, data, 10, {asset_id: 2})['data.0.x'], []);

const frames = [{timestamp_ns: '1700000000000000000', time_basis: 'frame', cameras: {'1': targets[0]}}];
for (const stage of ['multi_cam', 'cam_imu']) {
    const multi = renderAll([0], frames, 30, data, [10], stage, {asset_id: 1});
    assert.deepEqual(multi['data.0.x'], [50]);
    assert.deepEqual(multi['data.0.marker'].color, [5]);
    assert.equal(multi['data.1.x'], undefined);
    assert.deepEqual(multi['layout.xaxis.range'], [0, 640]);
    assert.match(multi['layout.title.text'], /Δt.*\\+0.000001 ms/);
    const missing = renderAll([0], frames, 30, data, [10], stage, {asset_id: 2});
    assert.deepEqual(missing['data.0.x'], []);
    assert.match(missing['layout.title.text'], /No matching sample/);
    assert.deepEqual(missing['data.0.customdata'], []);
}
frames[0].timestamp_ns = '1700000000005000001';
frames[0].time_basis = 'playback';
const earlier = renderAll([0], frames, 30, data, [10], 'cam_imu', {asset_id: 1});
assert.match(earlier['layout.title.text'], /Δt \\(playback\\) = -5.000000 ms/);
assert.equal(timestamp(0, frames), '1700000000005000001');
"""
        subprocess.run(
            [shutil.which("node"), "-e", script],
            check=True,
            capture_output=True,
            text=True,
        )
