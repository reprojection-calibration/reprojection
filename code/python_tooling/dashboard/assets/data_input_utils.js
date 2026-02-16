window.dataInputUtils = Object.assign({}, window.dataInputUtils, {
    getValidFrame: function (rawData, sensor, timestamps, frameIdx) {
        if (!timestamps || frameIdx == null) {
            return null;
        }
        const timestamp_i = BigInt(timestamps[frameIdx]);

        if (!rawData || !rawData[sensor] || !rawData[sensor]['frames'] || !rawData[sensor]['frames'][timestamp_i]) {
            return null;
        }

        return {
            frame: rawData[sensor]['frames'][timestamp_i],
            timestamp_i: timestamp_i
        };
    }
});
