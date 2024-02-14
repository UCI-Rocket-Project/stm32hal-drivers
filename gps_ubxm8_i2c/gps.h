#pragma once

#include "ubxPacket.h"
#include "ubxMessages.h"
#include <i2c.h>



class GPS {
    public:
        enum class State {
            REQUEST_NOT_SENT,
            POLLING_RESPONSE,
            RESPONSE_READY
        };
        enum class PollResult {
            POLL_FINISHED,
            NO_DATA,
            DATA_LEN_POLL_FAILED,
            DATA_RECEIVE_I2C_FAILED,
            DATA_RECEIVE_CHECKSUM_FAILED,
            RECEIVE_IN_PROGRESS
        };
        GPS();
        const State getState();
        const PollResult pollUpdate(I2C_HandleTypeDef* i2c);
        const UBX_NAV_PVT_PAYLOAD getSolution();
        void reset();

    private:
        UBXPacketReader packetReader;
        State state;
};

