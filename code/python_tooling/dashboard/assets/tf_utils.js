window.tfUtils = Object.assign({}, window.tfUtils, {
    buildAxisVector: function (origin, R, axis_id, scale) {
        const [x, y, z] = origin;

        const axisMap = {x: 0, y: 1, z: 2};
        const col = axisMap[axis_id];
        if (col === undefined) {
            throw new Error(`Invalid axis_id "${axis_id}". Must be 'x', 'y', or 'z'.`);
        }

        return [x + scale * R[0][col], y + scale * R[1][col], z + scale * R[2][col]];
    },

    addAxisVector: function (origin, axis, axis_id, patch) {
        const axisMap = {x: 0, y: 1, z: 2};
        const trace_id = axisMap[axis_id];
        if (trace_id === undefined) {
            throw new Error(`Invalid axis "${axis_id}". Must be 'x', 'y', or 'z'.`);
        }

        patch.assign(['data', trace_id, 'x'], [origin[0], axis[0]]);
        patch.assign(['data', trace_id, 'y'], [origin[1], axis[1]]);
        patch.assign(['data', trace_id, 'z'], [origin[2], axis[2]]);
    }
});


