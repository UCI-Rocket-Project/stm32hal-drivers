#pragma once

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
#endif

#ifndef SERIAL_TIMEOUT
#define SERIAL_TIMEOUT 10
#endif

class AltimeterMs5607Spi {
  public:
    struct Data {
        // pressure in Pascals
        double pressure;
        // temperature in degrees Celsius
        double temperature;
        // altitude in meters MSL
        double altitude;
    };

    enum State {
        RESET,
        IDLE,
        POLL_D1,
        POLL_D2,
        COMPLETE,
        ERROR
    };

    // oversampling rate setting, see datasheet for more details
    enum Rate {
        OSR256 = 0x40,
        OSR512 = 0x42,
        OSR1024 = 0x44,
        OSR2048 = 0x46,
        OSR4096 = 0x48
    };

    /**
     * @param hspi SPI bus handler
     * @param csPort chip select GPIO port
     * @param csPin chip select GPIO pin
     * @param misoPort MISO GPIO port
     * @param misoPin MISO GPIO pin
     * @param seaLevelPressure sea level pressure in hPa
     */
    AltimeterMs5607Spi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *misoPort, uint16_t misoPin, double seaLevelPressure);

    /**
     * @brief Resets altimeter
     * @retval Operation status, true for success
     */
    bool Reset();

    /**
     * @brief Initializes state machine, reads calibration coefficients
     * @retval Current state
     */
    State Init();

    /**
     * @brief Reads pressure and temperature data from the altimeter and calculates altitude
     * @retval Current state
     */
    State Convert(Rate rate);

    /**
     * @brief Get current data, resets system state to IDLE
     * @retval Current data
     */
    Data getData();

  private:
    SPI_HandleTypeDef *_hspi;
    GPIO_TypeDef *_csPort;
    uint16_t _csPin;
    GPIO_TypeDef *_misoPort;
    uint16_t _misoPin;
    double _seaLevelPressure;
    uint16_t _coefficients[8];  // factory calibration coefficients
    State state;
    uint32_t d1;  // raw pressure data
    uint32_t d2;  // raw temperature data
    Data data;
};
