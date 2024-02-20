/*
 * tc_max31855_spi.h
 *
 *  Created on: Jan 21, 2024
 *      Author: Alexandra Zhang Jiang
 */

#pragma once

#include "stm32f1xx_hal.h"

#ifndef SERIAL_TIMEOUT
#define SERIAL_TIMEOUT 10
#endif

class TcMax31855Spi {
  public:
    typedef struct Data {
        bool valid;
        // tc temperature data in degrees celsius
        float tcTemperature;
        // reference internal temperature in degrees celsius
        float internalTemperature;
    } Data;

    /**
     * @param hspi SPI bus handler
     * @param csPort chip select GPIO port
     * @param csPin chip select GPIO pin
     */
    TcMax31855Spi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);

    Data Read();

  private:
    // SPI bus handler
    SPI_HandleTypeDef *_hspi;
    // chip select GPIO port
    GPIO_TypeDef *_csPort;
    // chip select GPIO pin
    uint16_t _csPin;
};
