#include "ubxHelpers.h"
#include "ubxPacket.h"
#include "ubxMessages.h"
#include "gps.h"

#define I2C_BUFFER_SIZE 1024

GPS::GPS() {
    packetReader = UBXPacketReader();
    state = GPSState::REQUEST_NOT_SENT;
}

const GPSState GPS::getState() {
    return state;
}

const GPSPollResult GPS::pollUpdate(I2C_HandleTypeDef* i2c) {
    if(state == GPSState::REQUEST_NOT_SENT) {
        uint8_t message[4] = {0x01, 0x06, 0x00, 0x00};
        sendUBX(message, 4, &hi2c3);
        state = GPSState::POLLING_RESPONSE;
    }

    if(state == GPSState::POLLING_RESPONSE) {
        uint8_t lenBytes[2];
        uint16_t dataLen = 0; 
        uint8_t buffer[I2C_BUFFER_SIZE];

        HAL_StatusTypeDef dataLenReadStatus = HAL_I2C_Mem_Read(i2c, 0x42 << 1, 0xFD, 1, lenBytes, 2, 100);
        if(dataLenReadStatus != HAL_OK) {
            return GPSPollResult::DATA_LEN_POLL_FAILED;
        }

        dataLen = lenBytes[0] << 8 | lenBytes[1];
        if(dataLen==0) {
            return GPSPollResult::NO_DATA;
        }

        if(dataLen > I2C_BUFFER_SIZE) {
            dataLen = I2C_BUFFER_SIZE;
        }
        HAL_StatusTypeDef rcvStatus = HAL_I2C_Master_Receive(i2c, 0x42 << 1, buffer, dataLen, 100);
        if(rcvStatus != HAL_OK) {
            return GPSPollResult::DATA_RECEIVE_I2C_FAILED;
        }

        if(packetReader.isInProgress()) {
            for(uint16_t i = 0; i<dataLen; i++) {
                UBXPacketUpdateResult res = packetReader.update(buffer[i]);
                if(res == UBXPacketUpdateResult::CHECKSUM_FAILED) {
                    packetReader.reset();
                    return GPSPollResult::DATA_RECEIVE_CHECKSUM_FAILED;
                }
                if(packetReader.isComplete()) {
                    state = GPSState::RESPONSE_READY;
                    return GPSPollResult::POLL_OK;
                }
            }
        } else {
            for(uint16_t i = 0; i<dataLen; i++) {
                if(buffer[i] == 0xB5 && buffer[i+1] == 0x62) {
                    i+=2; // skip the header
                    while(i<dataLen && !packetReader.isComplete()) {
                        UBXPacketUpdateResult res = packetReader.update(buffer[i]);
                        if(res != UBXPacketUpdateResult::UPDATE_OK) {
                            packetReader.reset();
                            return GPSPollResult::DATA_RECEIVE_CHECKSUM_FAILED;
                        }
                        i++;
                    }
                }
            }

            if(packetReader.isComplete()) {
                state = GPSState::RESPONSE_READY;
                return GPSPollResult::POLL_OK;
            }
        }
    }

    return GPSPollResult::POLL_OK;
}

const UBX_NAV_SOL_PAYLOAD GPS::getSolution() {
    return *(UBX_NAV_SOL_PAYLOAD*)packetReader.getPayload();
}

void GPS::reset() {
    state = GPSState::REQUEST_NOT_SENT;
    packetReader.reset();
}