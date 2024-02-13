#pragma once
#include <stdint.h>

#pragma pack(push, 1)
struct UBX_NAV_PVT_PAYLOAD {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t reserved1[6];
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
};
#pragma pack(pop)

enum class GPSFixType {
    NO_FIX,
    DEAD_RECKONING,
    FIX_2D,
    FIX_3D,
    GNSS_AND_DEAD_RECKONING,
    TIME_ONLY_FIX
};

#pragma pack(push, 1)
struct UBX_NAV_STATUS_PAYLOAD {
    uint32_t iTOW;
    uint8_t gpsFix; // GPSFixType, but IDK if the enum will pack to 1 byte consistently
    uint8_t flags;
    uint8_t fixStat;
    uint8_t flags2;
    uint32_t ttff;
    uint32_t msss;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct UBX_NAV_SAT_PAYLOAD {
    uint32_t iTOW;
    uint8_t version;
    uint8_t numSvs;
    uint8_t reserved1[2];
    struct {
        uint8_t gnssId;
        uint8_t svId;
        uint8_t cno;
        int8_t elev;
        int16_t azim;
        int16_t prRes;
    } sats[16];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct UBX_NAV_SOL_PAYLOAD { // 0x01 0x06
    uint32_t iTOW;
    int32_t fTOW;
    int16_t week;
    uint8_t gpsFix; // GPSFixType, but IDK if the enum will pack to 1 byte consistently
    uint8_t flags;
    int32_t ecefX;
    int32_t ecefY;
    int32_t ecefZ;
    uint32_t pAcc;
    int32_t ecefVX;
    int32_t ecefVY;
    int32_t ecefVZ;
    uint32_t sAcc;
    uint16_t pDOP;
    uint8_t reserved1;
    uint8_t numSV;
    uint8_t reserved2[4];
};
#pragma pack(pop)