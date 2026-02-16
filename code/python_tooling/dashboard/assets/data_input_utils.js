window.dataInputUtils = Object.assign({}, window.dataInputUtils, {
    getTimestamps: function (metadata, sensorType, sensorName) {
        const timestamps = metadata?.[1]?.[sensorType]?.[sensorName];
        if (timestamps == null) {
            return null;
        }

        if (!Array.isArray(timestamps)) {
            return null;
        }

        return timestamps;
    },

    getValidFrame: function (rawData, sensorName, timestamps, frameIdx) {
        if (!timestamps || frameIdx == null || frameIdx < 0 || frameIdx >= timestamps.length) {
            return null;
        }
        const timestamp_i = BigInt(timestamps[frameIdx]);

        if (!rawData || !rawData[sensorName] || !rawData[sensorName]['frames'] || !rawData[sensorName]['frames'][timestamp_i]) {
            return null;
        }

        return {
            frame: rawData[sensorName]['frames'][timestamp_i], timestamp_i: timestamp_i
        };
    }
});
