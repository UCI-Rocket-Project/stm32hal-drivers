/*
 * tc_max31855_spi.cpp
 *
 *  Created on: Jan 31, 2024
 *      Author: Alexandra Zhang Jiang
 */

#include "tc_max31855_spi.h"

TcMax31855Spi::TcMax31855Spi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin) : _hspi(hspi), _csPort(csPort), _csPin(csPin) {}

TcMax31855Spi::Data TcMax31855Spi::Read() {
    TcMax31855Spi::Data data;
    data.valid = false;

    uint8_t buffer[4] = {0};
    uint8_t temp[4] = {0};
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);  // start SPI transaction

    if (HAL_SPI_Receive(_hspi, buffer, 4, SERIAL_TIMEOUT) != HAL_OK) return data;  // read 8-bits four times from miso line
    uint32_t dummy = _hspi->Intance->DR;                                           // flush

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    uint32_t d = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | ((uint32_t)buffer[3]);  // MSB

    // Detect fault
    if (d & 0x10003) return data;

    // TC temperature is bit 31 to bit 18
    int32_t tcTempBits = ((d >> 18) & 0x3FFF);             // isolate bits 31-18
    if (tcTempBits & (1 << 13)) tcTempBits |= 0xFFFFC000;  // Extend the sign in two's complement
    // Convert to float, based on the datasheet a unit represents 1/4 degrees
    data.tcTemperature = tcTempBits / 4.0f;

    // Internal temperature is bit 15 to bit 4
    int32_t inTempBits = ((d >> 4) & 0xFFF);
    if (inTempBits & (1 << 11)) inTempBits |= 0xFFFFF000;
    data.internalTemperature = inTempBits / 16.0f;  // a decimal unit represents 1/16 degrees

    data.valid = true;
    return data;
}
