#include "ubxHelpers.h"

#include "ubxMessages.h"
#include "ubxPacket.h"

#define I2C_BUFFER_SIZE 1024

static uint8_t magicBytes[2] = {0xB5, 0x62};

bool sendUBX(uint8_t* message, uint16_t len, I2C_HandleTypeDef* i2c) {
    uint8_t CK_A{0}, CK_B{0};

    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(i2c, 0x42 << 1, magicBytes, 2, 100);
    if (status != HAL_OK) {
        return false;
    }

    for (uint16_t i = 0; i < len; i++) {
        CK_A = CK_A + message[i];
        CK_B = CK_B + CK_A;
    }

    uint8_t CK[2] = {CK_A, CK_B};
    status = HAL_I2C_Master_Transmit(i2c, 0x42 << 1, message, len, 100);
    if (status != HAL_OK) {
        return false;
    }
    status = HAL_I2C_Master_Transmit(i2c, 0x42 << 1, CK, 2, 100);
    if (status != HAL_OK) {
        return false;
    }
    return true;
}
