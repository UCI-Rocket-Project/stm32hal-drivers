#pragma once

#include <math.h>

#include "stm32f1xx_hal.h"

#ifndef SERIAL_TIMEOUT
#define SERIAL_TIMEOUT 10
#endif

class AltimeterMs5607Spi {
  public:
    struct Data {
        // temperature data in degrees celsius
        double temperature = std::nan("");
        // pressure data in millibars or hPa
        double pressure = std::nan("");
        // altitude data in meters
        double altitude = std::nan("");
    };

    /**
     * @param hspi SPI bus handler
     * @param csPort chip select GPIO port
     * @param csPin chip select GPIO pin
     * @param misoPort MISO GPIO port
     * @param misoPin MISO GPIO pin
     * @param seaLevelPressure sea level pressure in hPa or millibars
     */
    AltimeterMs5607Spi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *misoPort, uint16_t misoPin, double seaLevelPressure);

    /**
     * @brief Resets the altimeter
     * @retval Operation status, 0 for success
     */
    int Reset();

    /**
     * @brief Initializes the altimeter by reading calibration coefficients
     * @retval Operation status, 0 for success
     */
    int Init();

    /**
     * @brief Reads pressure and temperature data from the altimeter and calculates altitude
     * @retval Altimeter data frame that contains pressure, temperature, and altitude
     */
    Data Read();

  private:
    // SPI bus handler
    SPI_HandleTypeDef *_hspi;
    // chip select GPIO port
    GPIO_TypeDef *_csPort;
    // chip select GPIO pin
    uint16_t _csPin;
    // MISO GPIO port
    GPIO_TypeDef *_misoPort;
    // MISO GPIO pin
    uint16_t _misoPin;
    // sea level pressure in hPa or millibars
    double _seaLevelPressure;
    // factory calibration coefficients
    int64_t _coefficients[6];
};
