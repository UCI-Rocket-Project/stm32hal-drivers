#pragma once
#include "i2c.h"
#include "ubxMessages.h"

bool sendUBX(uint8_t* message, uint16_t len, I2C_HandleTypeDef* i2c);

bool checkGPSFullNavInfo(UBX_NAV_PVT_PAYLOAD* pvt, I2C_HandleTypeDef* i2c);

bool checkGPSNavStatus(UBX_NAV_STATUS_PAYLOAD* status, I2C_HandleTypeDef* i2c);

bool checkGPSNavSatellites(UBX_NAV_SAT_PAYLOAD* sat, I2C_HandleTypeDef* i2c);