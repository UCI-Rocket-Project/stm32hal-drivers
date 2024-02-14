#pragma once

#include "ubxPacket.h"
#include "ubxMessages.h"
#include <i2c.h>


/**
 * Class for interfacing over i2c with any M8 GPS. 
 * Before this class is used, the GPS reset pin needs to be pulled high.
 * 
 * This class keeps its own finite state machine for abstracting away polling data availability and retrying requests.
 * An example usage is as follows:
 * ```
 *     GPS myGPS;
 *     while(1) {
 *        if(myGPS.pollUpdate(&i2c1) == GPS::PollResult::POLL_FINISHED) {
 *            UBX_NAV_PVT_PAYLOAD pvtData = myGPS.getSolution();
 *            myGPS.reset();
 *        }
 *     }
 * ```
 * 
 * After a packet is received, you _need_ to call the reset method. 
 * If you don't, `pollUpdate` will just do nothing and return a `POLL_ALREADY_FINISHED` response.
 * 
 * The datasheet is here: https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
 * The most relevant parts are "UBX Protocol" (section 32) and "DDC Port" (section 15).
 * 
*/
class GPS {
    public:
        enum class State {
            REQUEST_NOT_SENT,
            POLLING_RESPONSE,
            RESPONSE_READY
        };
        enum class PollResult {
            POLL_JUST_FINISHED,
            POLL_ALREADY_FINISHED,
            RECEIVE_IN_PROGRESS,
            NO_DATA,
            NO_UBX_DATA,
            DATA_LEN_POLL_FAILED,
            DATA_RECEIVE_I2C_FAILED,
            DATA_RECEIVE_CHECKSUM_FAILED
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

