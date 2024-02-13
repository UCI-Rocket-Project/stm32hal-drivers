#pragma once

#include "ubxPacket.h"
#include "ubxMessages.h"
#include <i2c.h>

enum class GPSState {
    REQUEST_NOT_SENT,
    POLLING_RESPONSE,
    RESPONSE_READY
};

enum class GPSPollResult {
    POLL_OK,
    NO_DATA,
    DATA_LEN_POLL_FAILED,
    DATA_RECEIVE_I2C_FAILED,
    DATA_RECEIVE_CHECKSUM_FAILED
};

class GPS {
    public:
        GPS();
        const GPSState getState();
        const GPSPollResult pollUpdate(I2C_HandleTypeDef* i2c);
        const UBX_NAV_SOL_PAYLOAD getSolution();
        void reset();

    private:
        UBXPacketReader packetReader;
        GPSState state;
};

