#pragma once
#include "i2c.h"
#include "ubxMessages.h"

bool sendUBX(uint8_t* message, uint16_t len, I2C_HandleTypeDef* i2c);
