#include "altimeter_ms5607_spi.h"

AltimeterMs5607Spi::AltimeterMs5607Spi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *misoPort, uint16_t misoPin, double seaLevelPressure)
    : _hspi(hspi), _csPort(csPort), _csPin(csPin), _misoPort(misoPort), _misoPin(misoPin), _seaLevelPressure(seaLevelPressure) {}

int AltimeterMs5607Spi::Reset() {
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    // reset, opcode 0x1E
    if (HAL_SPI_Transmit(_hspi, (uint8_t *)(const uint8_t[]){0x1E}, 1, SERIAL_TIMEOUT) != HAL_OK) return -1;
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    return 0;
}

int AltimeterMs5607Spi::Init() {
    for (int i = 0; i < 6; i++) {
        uint8_t buffer[2] = {0};
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
        // PROM read, opcode 0b1010xxx0
        uint8_t command[1] = {(uint8_t)(0b10100000 | ((i + 1) << 1))};
        if (HAL_SPI_Transmit(_hspi, command, 1, SERIAL_TIMEOUT) != HAL_OK) return -1;
        if (HAL_SPI_Receive(_hspi, buffer, 2, SERIAL_TIMEOUT) != HAL_OK) return -1;
        _coefficients[i] = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    }
    return 0;

    // TODO: check CRC
}

AltimeterMs5607Spi::Data AltimeterMs5607Spi::Read() {
    AltimeterMs5607Spi::Data data;

    uint32_t d1 = 0;
    uint32_t d2 = 0;

    {
        uint8_t buffer[3] = {0};
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
        // convert D1 pressure, 4096x oversampling, opcode 0x48
        if (HAL_SPI_Transmit(_hspi, (uint8_t *)(const uint8_t[]){0x48}, 1, SERIAL_TIMEOUT) != HAL_OK) return data;
        // poll for conversion complete
        int readStatus = 0;
        while (readStatus == 0) {
            readStatus = HAL_GPIO_ReadPin(_misoPort, _misoPin);
        }
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
        // read data, opcode 0x00
        if (HAL_SPI_Transmit(_hspi, (uint8_t *)(const uint8_t[]){0x00}, 1, SERIAL_TIMEOUT) != HAL_OK) return data;
        if (HAL_SPI_Receive(_hspi, buffer, 3, SERIAL_TIMEOUT) != HAL_OK) return data;
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
        d1 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);
    }

    {
        uint8_t buffer[3] = {0};
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
        // convert D2 temperature, 4096x oversampling, opcode 0x58
        if (HAL_SPI_Transmit(_hspi, (uint8_t *)(const uint8_t[]){0x58}, 1, SERIAL_TIMEOUT) != HAL_OK) return data;
        // poll for conversion complete
        int readStatus = 0;
        while (readStatus == 0) {
            readStatus = HAL_GPIO_ReadPin(_misoPort, _misoPin);
        }
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
        readStatus = 0;

        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
        // read data, opcode 0x00
        if (HAL_SPI_Transmit(_hspi, (uint8_t *)(const uint8_t[]){0x00}, 1, SERIAL_TIMEOUT) != HAL_OK) return data;
        if (HAL_SPI_Receive(_hspi, buffer, 3, SERIAL_TIMEOUT) != HAL_OK) return data;
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
        d2 = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);
    }

    int64_t dt = d2 - _coefficients[4] * 256;
    int64_t t = 2000 + dt * _coefficients[5] / 8388608;

    int64_t off = _coefficients[1] * 131072 + (_coefficients[3] * dt) / 64;
    int64_t sens = _coefficients[0] * 65536 + (_coefficients[2] * dt) / 128;

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
    data.pressure = (d1 * sens / 2097152 - off) / 3276800.0;
    data.altitude = 44307.7 * (1 - pow(data.pressure / _seaLevelPressure, 0.190284));
    return data;
}
