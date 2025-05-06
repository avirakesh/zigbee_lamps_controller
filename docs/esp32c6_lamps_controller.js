import * as m from "zigbee-herdsman-converters/lib/modernExtend";

export default {
    zigbeeModel: ["esp32c6"],
    model: "esp32c6",
    vendor: "HighOnH2O",
    description: "Automatically generated definition",
    extend: [
        m.deviceEndpoints({
            endpoints: {
                ep10: 10,
                ep11: 11,
                ep12: 12,
            },
        }),
        m.light({
            endpointNames: ["ep10", "ep11", "ep12"],
            effect: false,
            powerOnBehavior: false,
            colorTemp: { startup: false, range: [200, 333] },
            configureReporting: true,
        }),
    ],
    meta: {
        multiEndpoint: true,
        noOffTransitionWhenOff: (entity) => {
            return true;
        },
    },
};
