const AXIS_MAP = Object.freeze({
    x: 0, y: 1, z: 2
});

window.tfUtils = Object.assign({}, window.tfUtils, {

    buildAxisVector: function (origin, R, axis_id, scale) {
        const [x, y, z] = origin;

        const col = AXIS_MAP[axis_id];
        if (col === undefined) {
            throw new Error(`Invalid axis_id "${axis_id}". Must be 'x', 'y', or 'z'.`);
        }

        return [x + scale * R[0][col], y + scale * R[1][col], z + scale * R[2][col]];
    },

    addAxisVector: function (origin, axis, axis_id, patch) {
        const trace_id = AXIS_MAP[axis_id];
        if (trace_id === undefined) {
            throw new Error(`Invalid axis "${axis_id}". Must be 'x', 'y', or 'z'.`);
        }

        patch.assign(['data', trace_id, 'x'], [origin[0], axis[0]]);
        patch.assign(['data', trace_id, 'y'], [origin[1], axis[1]]);
        patch.assign(['data', trace_id, 'z'], [origin[2], axis[2]]);
    },

    toRotationMatrix: function (axis_angle) {
        const [rx, ry, rz] = axis_angle;

        const theta = Math.sqrt(rx * rx + ry * ry + rz * rz);
        if (theta < 1e-8) {
            return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
        }

        const kx = rx / theta;
        const ky = ry / theta;
        const kz = rz / theta;

        const c = Math.cos(theta);
        const s = Math.sin(theta);
        const v = 1 - c;

        // Rodrigues' rotation formula matrix notation - https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
        R = [[kx * kx * v + c, kx * ky * v - kz * s, kx * kz * v + ky * s], //
            [ky * kx * v + kz * s, ky * ky * v + c, ky * kz * v - kx * s], //
            [kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c]];

        return R
    },
});