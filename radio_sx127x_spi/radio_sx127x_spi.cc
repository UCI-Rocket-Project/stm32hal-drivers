#include "radio_sx127x_spi.h"

RadioSx127xSpi::RadioSx127xSpi(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *rstPort, uint16_t rstPin, GPIO_TypeDef *dio0Port, uint16_t dio0Pin, Mode mode,
                               RfPort rfPort, int frequency, int transmitPower, RampTime rampTime, Bandwidth bandwidth, CodingRate codingRate, SpreadingFactor spreadingFactor, int preambleLength,
                               int payloadLength, bool crcEnable)
    : _hspi(hspi),
      _csPort(csPort),
      _csPin(csPin),
      _rstPort(rstPort),
      _rstPin(rstPin),
      _dio0Port(dio0Port),
      _dio0Pin(dio0Pin),
      _mode(mode),
      _rfPort(rfPort),
      _frequency(frequency),
      _transmitPower(transmitPower),
      _rampTime(rampTime),
      _bandwidth(bandwidth),
      _codingRate(codingRate),
      _spreadingFactor(spreadingFactor),
      _preambleLength(preambleLength),
      _payloadLength(payloadLength),
      _crcEnable(crcEnable) {}

bool RadioSx127xSpi::Reset() {
    HAL_GPIO_WritePin(_rstPort, _rstPin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(_rstPort, _rstPin, GPIO_PIN_SET);
    HAL_Delay(5);
    return true;
}

RadioSx127xSpi::State RadioSx127xSpi::Init() {
    _state = IDLE;

    // sleep mode
    if (WriteRegister(0x01, 0b00000000) == false) _state = ERROR;

    // LoRa mode, set frequency mode
    if (WriteRegister(0x01, 0b10000000 | ((uint8_t)_rfPort << 3)) == false) _state = ERROR;

    // set carrier frequency
    uint32_t f = (uint64_t)_frequency * 524288ULL / 32000000ULL;
    if (WriteRegister(0x06, (uint8_t)(f >> 16)) == false) _state = ERROR;
    if (WriteRegister(0x07, (uint8_t)(f >> 8)) == false) _state = ERROR;
    if (WriteRegister(0x08, (uint8_t)f) == false) _state = ERROR;

    // use PA_BOOST, set transmit power
    if (WriteRegister(0x09, 0b10000000 | (uint8_t)(_transmitPower - 2)) == false) _state = ERROR;

    // set PA ramp time
    if (WriteRegister(0x0A, (uint8_t)_rampTime) == false) _state = ERROR;

    // configure over-current protection
    if (WriteRegister(0x0B, 0b00111011) == false) _state = ERROR;

    // configure LNA
    if (WriteRegister(0x0C, 0b00100000) == false) _state = ERROR;

    // set FIFO TX base address
    if (WriteRegister(0x0E, 0x80) == false) _state = ERROR;

    // set FIFO RX base address
    if (WriteRegister(0x0F, 0x00) == false) _state = ERROR;

    // set bandwidth, set coding rate, use implicit header mode
    if (WriteRegister(0x1D, 0b00000001 | ((uint8_t)_bandwidth << 4) | ((uint8_t)_codingRate << 1)) == false) _state = ERROR;

    // set spreading factor, use non-continuous mode, set CRC
    if (WriteRegister(0x0E, ((uint8_t)_spreadingFactor << 4) | ((uint8_t)_crcEnable << 2)) == false) _state = ERROR;

    // set preamble length
    if (WriteRegister(0x20, (uint8_t)(_preambleLength >> 8)) == false) _state = ERROR;
    if (WriteRegister(0x21, (uint8_t)_preambleLength) == false) _state = ERROR;

    // set payload length
    if (WriteRegister(0x22, (uint8_t)_payloadLength) == false) _state = ERROR;

    // if radio is only transmitting, set DIO0 to TxDone IRQ, else DIO0 is RxDone IRQ (default)
    if (_mode == TX_ONLY) {
        if (WriteRegister(0x40, 0b01000000) == false) _state = ERROR;
    }

    // standby mode
    if (WriteRegister(0x01, 0b10000001 | ((uint8_t)_rfPort << 3)) == false) _state = ERROR;

    return _state;
}

RadioSx127xSpi::State RadioSx127xSpi::Transmit(const uint8_t *payload) {
    if (_mode != TXRX && _mode != TX_ONLY) {
        _state = ERROR;
        return _state;
    }

    switch (_state) {
        case IDLE: {
            _state = TX;

            // set FIFO address pointer
            if (WriteRegister(0x0D, 0x80) == false) _state = ERROR;

            // write payload to FIFO
            HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
            if (HAL_SPI_Transmit(_hspi, (uint8_t *)(const uint8_t[]){0x80}, 1, SERIAL_TIMEOUT) != HAL_OK) _state = ERROR;
            if (HAL_SPI_Transmit(_hspi, (uint8_t *)payload, _payloadLength, SERIAL_TIMEOUT) != HAL_OK) _state = ERROR;
            HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

            // if radio is bidirectional, set DIO0 to TxDone IRQ, else DIO0 should already be in this mode
            if (_mode == TXRX) {
                if (WriteRegister(0x40, 0b01000000) == false) _state = ERROR;
            }

            // start TX
            if (WriteRegister(0x01, 0b10000011 | ((uint8_t)_rfPort << 3)) == false) _state = ERROR;

            break;
        }

        case TX: {
            if (HAL_GPIO_ReadPin(_dio0Port, _dio0Pin) == GPIO_PIN_SET) {
                _state = IDLE;

                // clear TxDone IRQ
                if (WriteRegister(0x12, 0b00001000) == false) _state = ERROR;
            }
            break;
        }

        default:
            break;
    }
    return _state;
}

RadioSx127xSpi::State RadioSx127xSpi::Receive(uint8_t *payload, int *rssi) {
    if (_mode != TXRX && _mode != RX_ONLY) {
        _state = ERROR;
        return _state;
    }

    switch (_state) {
        case IDLE: {
            _state = RX;

            if (_mode == TXRX) {
                // if radio is bidirectional, set DIO0 to RxDone IRQ, else DIO0 should already be in this mode
                if (WriteRegister(0x40, 0b00000000) == false) _state = ERROR;
            }

            // start RX single
            if (WriteRegister(0x01, 0b10000110 | ((uint8_t)_rfPort << 3)) == false) _state = ERROR;

            break;
        }

        case RX: {
            if (HAL_GPIO_ReadPin(_dio0Port, _dio0Pin) == GPIO_PIN_SET) {
                _state = IDLE;

                // clear RxDone IRQ
                if (WriteRegister(0x12, 0b01000000) == false) _state = ERROR;
            }

            // set FIFO address pointer
            if (WriteRegister(0x0D, 0x00) == false) _state = ERROR;

            // read payload from FIFO
            HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
            if (HAL_SPI_Transmit(_hspi, (uint8_t *)(const uint8_t[]){0x00}, 1, SERIAL_TIMEOUT) != HAL_OK) _state = ERROR;
            if (HAL_SPI_Receive(_hspi, (uint8_t *)payload, _payloadLength, SERIAL_TIMEOUT) != HAL_OK) _state = ERROR;
            HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

            // read RSSI
            uint8_t data;
            if (ReadRegister(0x1A, &data) == false) _state = ERROR;
            if (_rfPort == HF) {
                *rssi = data - 157;
            } else if (_rfPort == LF) {
                *rssi = data - 164;
            }
        }

        default:
            break;
    }
    return _state;
}

bool RadioSx127xSpi::ReadRegister(uint8_t address, uint8_t *data) {
    bool status = true;
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(_hspi, &address, 1, SERIAL_TIMEOUT) != HAL_OK) status = false;
    if (HAL_SPI_Receive(_hspi, data, 1, SERIAL_TIMEOUT) != HAL_OK) status = false;
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    return status;
}

bool RadioSx127xSpi::WriteRegister(uint8_t address, uint8_t data) {
    bool status = true;
    uint8_t payload[2] = {(uint8_t)(address | 0x80), data};
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(_hspi, payload, 2, SERIAL_TIMEOUT) != HAL_OK) status = false;
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
    return status;
}
