#pragma once

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
#endif

#ifndef SERIAL_TIMEOUT
#define SERIAL_TIMEOUT 10
#endif

class RadioSx127xSpi {
  public:
    enum State { IDLE, TX, RX, ERROR };

    enum Mode {
        // bidirectional
        TXRX,
        // transmit only
        TX_ONLY,
        // receive only
        RX_ONLY
    };

    enum RfPort {
        // high frequency port 779-1020 MHz
        HF = 0,
        // low frequency port 137-525 MHz
        LF = 1
    };

    enum RampTime {
        // 3.4 ms
        RT3M4S = 0,
        // 2 ms
        RT2MS = 1,
        // 1 ms
        RT1MS = 2,
        // 500 us
        RT500US = 3,
        // 250 us
        RT250US = 4,
        // 125 us
        RT125US = 5,
        // 100 us
        RT100US = 6,
        // 62 us
        RT62US = 7,
        // 50 us
        RT50US = 8,
        // 40 us
        RT40US = 9,
        // 31 us
        RT31US = 10,
        // 25 us
        RT25US = 11,
        // 20 us
        RT20US = 12,
        // 15 us
        RT15US = 13,
        // 12 us
        RT12US = 14,
        // 10 us
        RT10US = 15
    };

    enum Bandwidth {
        // 7.8 kHz
        BW7K8HZ = 0,
        // 10.4 kHz
        BW10K4HZ = 1,
        // 15.6 kHz
        BW15K6HZ = 2,
        // 20.8 kHz
        BW20K8HZ = 3,
        // 31.25 kHz
        BW31K25HZ = 4,
        // 41.7 kHz
        BW41K7HZ = 5,
        // 62.5 kHz
        BW62K5HZ = 6,
        // 125 kHz
        BW125KHZ = 7,
        // 250 kHz
        BW250KHZ = 8,
        // 500 kHz
        BW500KHZ = 9
    };

    enum CodingRate {
        // 4/5
        CR45 = 1,
        // 4/6
        CR46 = 2,
        // 4/7
        CR47 = 3,
        // 4/8
        CR48 = 4
    };

    enum SpreadingFactor { SF6 = 6, SF7 = 7, SF8 = 8, SF9 = 9, SF10 = 10, SF11 = 11, SF12 = 12 };

    /**
     * @param hspi SPI bus handler
     * @param csPort chip select GPIO port
     * @param csPin chip select GPIO pin
     * @param rstPort reset GPIO port
     * @param rstPin reset GPIO pin
     * @param dio0Port DIO0 GPIO port
     * @param dio0Pin DIO0 GPIO pin
     * @param frequency RF center frequency in Hz, valid range 137000000-525000000, 779000000-1020000000
     * @param transmitPower transmit power in dBm, valid range 2-15
     * @param rampTime PA ramp time
     * @param bandwidth signal bandwidth
     * @param codingRate error coding rate
     * @param spreadingFactor spreading factor rate
     * @param preambleLength preamble length in symbols, valid range 6-65535
     * @param payloadLength packet length in bytes, valid range 1-255
     * @param crcEnable enable CRC
     */
    RadioSx127xSpi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *rstPort, uint16_t rstPin, GPIO_TypeDef *dio0Port, uint16_t dio0Pin, Mode mode, RfPort rfPort,
                   int frequency, int transmitPower, RampTime rampTime, Bandwidth bandwidth, CodingRate codingRate, SpreadingFactor spreadingFactor, int preambleLength, int payloadLength,
                   bool crcEnable);

    /**
     * @brief Resets radio
     * @retval Operation status, true for success
     */
    bool Reset();

    /**
     * @brief Initializes and configures radio
     * @retval Current state
     */
    State Init();

    /**
     * @brief Transmits payload
     * @param payload pointer to payload buffer, must be the same length as specified by payloadLength
     * @retval Current state, returns to IDLE when transmit complete
     */
    State Transmit(const uint8_t *payload);

    /**
     * @brief Listens and receives one packet
     * @param payload pointer to payload buffer, must be the same length as specified by payloadLength
     * @param rssi received signal strength indication, this is an output
     * @retval Current state, returns to IDLE when receive complete
     */
    State Receive(uint8_t *payload, int *rssi);

  private:
    /**
     * @retval Operation status, true for success
     */
    bool ReadRegister(uint8_t address, uint8_t *data);

    /**
     * @retval Operation status, true for success
     */
    bool WriteRegister(uint8_t address, uint8_t data);

    SPI_HandleTypeDef *_hspi;
    GPIO_TypeDef *_csPort;
    uint16_t _csPin;
    GPIO_TypeDef *_rstPort;
    uint16_t _rstPin;
    GPIO_TypeDef *_dio0Port;
    uint16_t _dio0Pin;
    Mode _mode;
    RfPort _rfPort;
    int _frequency;
    int _transmitPower;
    RampTime _rampTime;
    Bandwidth _bandwidth;
    CodingRate _codingRate;
    SpreadingFactor _spreadingFactor;
    int _preambleLength;
    int _payloadLength;
    bool _crcEnable;
    State _state;
};
