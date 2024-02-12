/*
 * W25Q128JV_MEMORY.h
 *
 *  Created on: Jan 1, 2024
 *      Author: rober
 */
#pragma once

#include <cmath>

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#endif

#ifndef SPI_Timeout
#define SPI_Timeout 10
#endif

class MemoryW25q1128jvSpi {
  public:
#pragma pack(push, 1)
    struct TelemetryData {
        uint8_t type = 0x00;
        uint32_t timestamp = 0xFFFFFFFF;             // ms since startup
        uint8_t state = 0x00;                        // state for AFS, unused on ECU
        int16_t imuGyroscopeX = 0xFFFF;              // 0.061 deg/s per LSB
        int16_t imuGyroscopeY = 0xFFFF;              // 0.061 deg/s per LSB
        int16_t imuGyroscopeZ = 0xFFFF;              // 0.061 deg/s per LSB
        int16_t imuAccelerometerX = 0xFFFF;          // 0.00073g per LSB
        int16_t imuAccelerometerY = 0xFFFF;          // 0.00073g per LSB
        int16_t imuAccelerometerZ = 0xFFFF;          // 0.00073g per LSB
        int16_t imuMagnetometerX = 0xFFFF;           // 0.063 uT per LSB
        int16_t imuMagnetometerY = 0xFFFF;           // 0.063 uT per LSB
        int16_t imuMagnetometerZ = 0xFFFF;           // 0.063 uT per LSB
        int16_t altimeterTemperature = 0xFFFF;       // 10^-2 C
        int32_t altimeterAltitude = 0xFFFFFFFF;      // cm
        int32_t gnssEcefPositionX = 0xFFFFFFFF;      // cm
        int32_t gnssEcefPositionY = 0xFFFFFFFF;      // cm
        int32_t gnssEcefPositionZ = 0xFFFFFFFF;      // cm
        uint32_t gnssPositionAccuracy = 0xFFFFFFFF;  // cm
        int32_t gnssEcefVelocityX = 0xFFFFFFFF;      // cm/s
        int32_t gnssEcefVelocityY = 0xFFFFFFFF;      // cm/s
        int32_t gnssEcefVelocityZ = 0xFFFFFFFF;      // cm/s
        uint32_t gnssVelocityAccuracy = 0xFFFFFFFF;  // cm/s
        uint16_t crc = 0x0000;
    };
#pragma pack(pop)

#pragma pack(push, 1)
    struct AfsState {
        bool armPinState : 1;       // 0: unarmed, 1: armed
        bool drogueContinuity : 1;  // 0: no continuity, 1: continuity
        bool mainContinuity : 1;    // 0: no continuity, 1: continuity
        uint16_t : 1;               // reserved
        /**
         * 0x0: standby
         * 0x1: armed
         * 0x2: boost
         * 0x3: coast
         * 0x4: apogee
         * 0x5: drogue fired
         * 0x6: drogue opened
         * 0x7 drogue failure
         * 0x8: main fired
         * 0x9: main opened
         * 0xA: main failure
         * 0xB: land
         */
        uint8_t state : 4;
    };
#pragma pack(pop)

    /**
     * @param _spi 		 SPI bus handler
     * @param _csPort 	 chip select GPIO port
     * @param _csPin 	 chip select GPIO pin
     * @param _holdPort  Write HOLD GPIO port
     * @param _holdPin 	 Write HOLD GPIO pin
     * @param _wpPort 	 Write Protect GPIO port
     * @param _wpPin	 Write Protect GPIO pin
     */
    MemoryW25q1128jvSpi(SPI_HandleTypeDef *_spi, GPIO_TypeDef *_csPort, uint16_t _csPin, GPIO_TypeDef *_holdPort, uint16_t _holdPin, GPIO_TypeDef *_wpPort, uint16_t);

    /**
     * @brief Initialises the memory by reading and incrementing to the first non-written page
     * @retval Operation status, 1 for success
     */
    uint8_t Init();

    /**
     * @brief Reads the device ID of the memory/
     * 		  Should output 0xEF for manufacturer ID,
     * 		  Should output 0x90 for Device ID
     */
    uint16_t DeviceID();

    /**
     * @brief Reads the first status register from memort
     * 		  2 inputs
     * 			Reg_check, Status Register bit to check status of
     * 		  	Reg_until, wait until Status register bit = Reg_until value
     * @retval Operation status, 1 for success
     */
    uint8_t Read_Status_Reg1(uint8_t Reg_check, uint8_t Reg_until);

    /**
     * @brief Erases all data on chip
     * @retval Operation status, 1 for success
     */
    uint8_t Chip_Erase();

    /**
     * @brief Writes Telemetrydata stuct into TelemetryData Data_Bundle[4] array
     * @retval Operation status, 1 for success
     */
    uint8_t Chip_Write(TelemetryData data);

    /**
     * @brief Reads read_addess page data and returns eturns pointer to Data_Bundle array
     *
     */
    TelemetryData *Chip_Read(uint32_t read_address);

    // Data bundle array consisting of 4, 64 byte, data packages outlined in struct Telemetrydata
    MemoryW25q1128jvSpi::TelemetryData Data_Bundle[4];

  private:
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *csPort;
    uint16_t csPin;
    GPIO_TypeDef *holdPort;
    uint16_t holdPin;
    GPIO_TypeDef *wpPort;
    uint16_t wpPin;
    uint8_t Data_Bundle_Size = 0;
    uint32_t address = 0x00000000;
};
