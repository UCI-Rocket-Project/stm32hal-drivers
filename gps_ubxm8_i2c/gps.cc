#include "gps.h"

#include "ubxHelpers.h"
#include "ubxMessages.h"
#include "ubxPacket.h"

#define I2C_BUFFER_SIZE 1024

GPS::GPS() {
    packetReader = UBXPacketReader();
    state = GPS::State::REQUEST_NOT_SENT;
}

const GPS::State GPS::getState() { return state; }

/**
 * Sends a request for position data if none are currently pending.
 * Then, checks if there is data available from the GPS. If there is none,
 * it just returns a NO_DATA response. If there is data, it reads up to 1024 bytes (configurable at the top of gps.cpp)
 * After reading, it will return one of these:
 * - RECEIVE_IN_PROGRESS: it found a valid header for the data we're looking for (UBX packet) but hasn't read all of it yet.
 * - NO_UBX_DATA: it successfully read data but there was no header for the data we're looking for (UBX packet)
 * - POLL_JUST_FINISHED: it got data. Now you can read it with GPS::getSolution.
 *
 * There are also special return types for different types of error, that are hopefully verbose enough to explain themselves.
 */
const GPS::PollResult GPS::pollUpdate(I2C_HandleTypeDef* i2c) {
    if (state == GPS::State::REQUEST_NOT_SENT) {
        uint8_t message[4] = {0x01, 0x07, 0x00, 0x00};
        sendUBX(message, 4, &hi2c3);
        state = GPS::State::POLLING_RESPONSE;
    }

    if (state == GPS::State::POLLING_RESPONSE) {
        uint8_t lenBytes[2];
        uint16_t dataLen = 0;
        uint8_t buffer[I2C_BUFFER_SIZE];

        HAL_StatusTypeDef dataLenReadStatus = HAL_I2C_Mem_Read(i2c, 0x42 << 1, 0xFD, 1, lenBytes, 2, 100);
        if (dataLenReadStatus != HAL_OK) {
            return GPS::PollResult::DATA_LEN_POLL_FAILED;
        }

        dataLen = lenBytes[0] << 8 | lenBytes[1];
        if (dataLen == 0) {
            return GPS::PollResult::NO_DATA;
        }

        if (dataLen > I2C_BUFFER_SIZE) {
            dataLen = I2C_BUFFER_SIZE;
        }
        HAL_StatusTypeDef rcvStatus = HAL_I2C_Master_Receive(i2c, 0x42 << 1, buffer, dataLen, 100);
        if (rcvStatus != HAL_OK) {
            return GPS::PollResult::DATA_RECEIVE_I2C_FAILED;
        }

        if (packetReader.isInProgress()) {
            for (uint16_t i = 0; i < dataLen; i++) {
                UBXPacketUpdateResult res = packetReader.update(buffer[i]);
                if (res == UBXPacketUpdateResult::CHECKSUM_FAILED) {
                    packetReader.reset();
                    return GPS::PollResult::DATA_RECEIVE_CHECKSUM_FAILED;
                }
                if (packetReader.isComplete()) {
                    state = GPS::State::RESPONSE_READY;
                    return GPS::PollResult::POLL_JUST_FINISHED;
                }
            }
            return GPS::PollResult::RECEIVE_IN_PROGRESS;
        } else {
            for (uint16_t i = 0; i < dataLen; i++) {
                if (buffer[i] == 0xB5 && buffer[i + 1] == 0x62) {
                    i += 2;  // skip the header
                    while (i < dataLen && !packetReader.isComplete()) {
                        UBXPacketUpdateResult res = packetReader.update(buffer[i]);
                        if (res != UBXPacketUpdateResult::UPDATE_OK) {
                            packetReader.reset();
                            return GPS::PollResult::DATA_RECEIVE_CHECKSUM_FAILED;
                        }
                        i++;
                    }
                }
            }

            if (!packetReader.isInProgress()) {
                return GPS::PollResult::NO_UBX_DATA;
            }

            if (packetReader.isComplete()) {
                state = GPS::State::RESPONSE_READY;
                return GPS::PollResult::POLL_JUST_FINISHED;
            }
            return GPS::PollResult::RECEIVE_IN_PROGRESS;
        }
    }
    return GPS::PollResult::POLL_ALREADY_FINISHED;
}

/**
 * Returns GPS position solution info. This will only give valid data
 * if it's called after `pollUpdate` returns a POLL_JUST_FINISHED response,
 * which will put the GPS's state in RESPONSE_READY mode. If you call this before that,
 * the data in the struct is undefined.
 *
 * After calling this and using the info returned, getting the next
 * packet requires you to call `reset` and `pollUpdate` again.
 */
const UBX_NAV_PVT_PAYLOAD GPS::getSolution() { return *(UBX_NAV_PVT_PAYLOAD*)packetReader.getPayload(); }

/**
 * Puts the GPS state back to its initial value so that `pollUpdate` knows
 * it needs to send a new data request.
 */
void GPS::reset() {
    state = GPS::State::REQUEST_NOT_SENT;
    packetReader.reset();
}
