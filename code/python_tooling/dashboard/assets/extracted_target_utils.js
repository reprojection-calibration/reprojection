window.extractedTargetUtils = Object.assign({}, window.extractedTargetUtils, {
    buildExtractedTargetPatch: function (points, reprojection_error, max_error) {
        const patch = new dash_clientside.Patch();

        patch.assign(['data', 0, 'x'], points.map(p => p[0]));
        patch.assign(['data', 0, 'y'], points.map(p => p[1]));

        // If reprojection error is not available for this frame return markers to the default coloring and size
        if (!reprojection_error) {
            patch.assign(['data', 0, 'marker'], {
                size: 12
            });

            return patch;
        }

        patch.assign(['data', 0, 'marker'], {
            size: 12,
            color: reprojection_error.map(p => Math.sqrt(p[0] * p[0] + p[1] * p[1])),
            colorscale: "Bluered",
            cmin: 0,
            cmax: max_error,
            showscale: true
        });

        return patch;
    },
});