#include "altimeter_ms5607_spi.h"

#include <cmath>

AltimeterMs5607Spi::AltimeterMs5607Spi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *misoPort, uint16_t misoPin, double seaLevelPressure)
    : _hspi(hspi), _csPort(csPort), _csPin(csPin), _misoPort(misoPort), _misoPin(misoPin), _seaLevelPressure(seaLevelPressure) {}

bool AltimeterMs5607Spi::Reset() {
    bool opStatus = true;

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    // reset, opcode 0x1E
    if (HAL_SPI_Transmit(_hspi, (uint8_t *)(const uint8_t[]){0x1E}, 1, SERIAL_TIMEOUT) != HAL_OK) opStatus = false;
    // poll for reset complete (approx 2.8ms)
    GPIO_PinState readStatus = GPIO_PIN_RESET;
    while (readStatus == GPIO_PIN_RESET) {
        readStatus = HAL_GPIO_ReadPin(_misoPort, _misoPin);
    }
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    return opStatus;
}

AltimeterMs5607Spi::State AltimeterMs5607Spi::Init() {
    state = AltimeterMs5607Spi::State::IDLE;

    // read factory coefficients
    for (int i = 0; i < 8; i++) {
        uint8_t buffer[2];
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
        // PROM read, opcode 0b1010xxx0
        uint8_t command[1] = {(uint8_t)(0b10100000 | (i << 1))};
        if (HAL_SPI_Transmit(_hspi, command, 1, SERIAL_TIMEOUT) != HAL_OK) state = AltimeterMs5607Spi::State::ERROR;
        if (HAL_SPI_Receive(_hspi, buffer, 2, SERIAL_TIMEOUT) != HAL_OK) state = AltimeterMs5607Spi::State::ERROR;
        _coefficients[i] = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    }

    // CRC algorithm copied from TE application note AN520
    uint16_t crc = 0xF & _coefficients[7];
    uint16_t crcActual = 0;
    _coefficients[7] = 0xFF00 & _coefficients[7];
    for (int i = 0; i < 16; i++) {
        crcActual ^= i % 2 == 0 ? _coefficients[i >> 1] >> 8 : _coefficients[i >> 1] & 0x00FF;
        for (int j = 0; j < 8; j++) {
            crcActual = crcActual & 0x8000 ? (crcActual << 1) ^ 0x3000 : (crcActual << 1);
        }
    }
    crcActual = crcActual >> 12;
    if (crc != crcActual) state = AltimeterMs5607Spi::State::ERROR;

    return state;
}

AltimeterMs5607Spi::State AltimeterMs5607Spi::Convert(AltimeterMs5607Spi::Rate rate) {
    switch (state) {
        case AltimeterMs5607Spi::State::IDLE: {
            state = AltimeterMs5607Spi::State::POLL_D1;

            HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
            // convert D1 pressure, the value of the rate enum is the opcode of the corresponding conversion
            uint8_t command = (uint8_t)rate;
            if (HAL_SPI_Transmit(_hspi, &command, 1, SERIAL_TIMEOUT) != HAL_OK) state = AltimeterMs5607Spi::State::ERROR;
            break;
        }

        case AltimeterMs5607Spi::State::POLL_D1: {
            if (HAL_GPIO_ReadPin(_misoPort, _misoPin) == GPIO_PIN_RESET) break;
            HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

            state = AltimeterMs5607Spi::State::POLL_D2;

            uint8_t buffer[3];
            HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
            // read data, opcode 0x00
            if (HAL_SPI_Transmit(_hspi, (uint8_t *)(const uint8_t[]){0x00}, 1, SERIAL_TIMEOUT) != HAL_OK) state = AltimeterMs5607Spi::State::ERROR;
            if (HAL_SPI_Receive(_hspi, buffer, 3, SERIAL_TIMEOUT) != HAL_OK) state = AltimeterMs5607Spi::State::ERROR;
            HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
            d1 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);

            HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
            // convert D2 temperature, the value of the rate enum is the opcode of the corresponding conversion plus 0x10
            uint8_t command = (uint8_t)rate + 0x10;
            if (HAL_SPI_Transmit(_hspi, &command, 1, SERIAL_TIMEOUT) != HAL_OK) state = AltimeterMs5607Spi::State::ERROR;
            break;
        }

        case AltimeterMs5607Spi::State::POLL_D2: {
            if (HAL_GPIO_ReadPin(_misoPort, _misoPin) == GPIO_PIN_RESET) break;
            HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

            state = AltimeterMs5607Spi::State::COMPLETE;
            uint8_t buffer[3];
            HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
            // read data, opcode 0x00
            if (HAL_SPI_Transmit(_hspi, (uint8_t *)(const uint8_t[]){0x00}, 1, SERIAL_TIMEOUT) != HAL_OK) state = AltimeterMs5607Spi::State::ERROR;
            if (HAL_SPI_Receive(_hspi, buffer, 3, SERIAL_TIMEOUT) != HAL_OK) state = AltimeterMs5607Spi::State::ERROR;
            HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
            d2 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);

            // calculate temperature, pressure, and altitude, see datasheet for formulas used
            int64_t dt = d2 - (int64_t)_coefficients[5] * 256;
            int64_t t = 2000 + dt * (int64_t)_coefficients[6] / 8388608;
            int64_t off = (int64_t)_coefficients[2] * 131072 + ((int64_t)_coefficients[4] * dt) / 64;
            int64_t sens = (int64_t)_coefficients[1] * 65536 + ((int64_t)_coefficients[3] * dt) / 128;

            // low temperature correction
            int64_t t2 = 0;
            int64_t off2 = 0;
            int64_t sens2 = 0;
            if (t < 2000) {
                int64_t a = t - 2000;
                t2 = dt * dt / 2147483648;
                off2 = 61 * a * a / 16;
                sens2 = 2 * a * a;

                if (t < -1500) {
                    int64_t b = t + 1500;
                    off2 += 15 * b * b;
                    sens2 += 8 * b * b;
                }
            }
            t -= t2;
            off -= off2;
            sens -= sens2;

            data.temperature = t / 100.0;
            data.pressure = (d1 * sens / 2097152 - off) / 32768.0;
            data.altitude = 44307.7 * (1 - std::pow(data.pressure / 100.0 / _seaLevelPressure, 0.190284));
            break;
        }

        default:
            break;
    }
    return state;
}

AltimeterMs5607Spi::Data AltimeterMs5607Spi::getData() {
    state = State::IDLE;
    return data;
}
