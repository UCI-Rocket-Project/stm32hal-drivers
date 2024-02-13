#pragma once

#include <stdint.h>

enum class UBXPacketUpdateResult {
    UPDATE_OK,
    CHECKSUM_FAILED,
    PACKET_ALREADY_COMPLETE
};

class UBXPacketReader {
    public:
        UBXPacketReader();
        UBXPacketUpdateResult update(uint8_t byte);
        bool isComplete();
        bool isInProgress();
        void* getPayload();
        uint8_t getPayloadLength();
        uint8_t getMessageClass();
        uint8_t getMessageId();
        void reset();

    private:
        uint8_t payload[512];
        uint8_t payloadLength;
        uint8_t packetIndex;
        uint8_t messageClass;
        uint8_t messageId;
        uint8_t ckA;
        uint8_t ckB;
        bool complete;
        bool inProgress;
};
