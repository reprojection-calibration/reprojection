window.dataInputUtils = Object.assign({}, window.dataInputUtils, {
    getTimestamps: function (metadata, sensorType, sensorName, frameIdx) {
        if (!metadata || !sensorType || !sensorName || frameIdx == null) {
            return null;
        }

        const timestamps = metadata?.[1]?.[sensorType]?.[sensorName];
        if (timestamps == null) {
            return null;
        }

        if (!Array.isArray(timestamps)) {
            throw new Error(`Expected "timestamps" for sensor "${sensor}" to be an array, but got type "${typeof timestamps}".`);
        }

        if (frameIdx < 0 || frameIdx >= timestamps.length) {
            throw new Error(`Invalid frame index ${frameIdx}. It must be between 0 and ${timestamps.length - 1}.`);
        }

        const timestamp_i = BigInt(timestamps[frameIdx]);

        return {
            timestamps: timestamps, timestamp_i: timestamp_i
        };
    },

    getValidFrame: function (rawData, sensorName, timestamp) {
        if (!rawData || !sensorName || timestamp == null) {
            return null;
        }

        const frame = rawData?.[sensorName]?.frames?.[timestamp];
        if (!frame) {
            return null;
        }

        return frame
    }
});
