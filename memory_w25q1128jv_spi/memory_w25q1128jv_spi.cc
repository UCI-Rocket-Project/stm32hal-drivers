#include <memory_w25q1128jv_spi.h>

MemoryW25q1128jvSpi::MemoryW25q1128jvSpi(SPI_HandleTypeDef *_spi, GPIO_TypeDef *_csPort, uint16_t _csPin, GPIO_TypeDef *_holdPort, uint16_t _holdPin, GPIO_TypeDef *_wpPort, uint16_t _wpPin)

    : spi(_spi), csPort(_csPort), csPin(_csPin), holdPort(_holdPort), holdPin(_holdPin), wpPort(_wpPort), wpPin(_wpPin) {}

uint8_t MemoryW25q1128jvSpi::Init() {
    // making sure control pins are always high
    HAL_GPIO_WritePin(wpPort, wpPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(holdPort, holdPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

    // Data holder for reading address
    afsTelemetryData _init_Data;
    // Reads Chip ID, if chip ID is not W25Q128JV_MEMORY, then return 0
    if (DeviceID() != 0xEF17) return 0;

    // while loop to read through every memory page
    // Result, address starts in the first empty page
    while (1) {
        // Reads a page of memory
        Chip_Read(address);afsTelemetryData;
        address++;
    }
    return 1;
}

uint16_t MemoryW25q1128jvSpi::DeviceID() {
    // making sure control pins are always high
    HAL_GPIO_WritePin(wpPort, wpPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(holdPort, holdPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

    // input instruction hex + 3 dummy bytes
    uint8_t in[4] = {0x90, 0x00, 0x00, 0x00};
    // output array of bytes
    uint8_t out[2] = {0};

    // SPI protocol
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(spi, in, 4, 10) != HAL_OK) return 0;
    if (HAL_SPI_Receive(spi, out, 2, 10) != HAL_OK) return 0;
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

    // returns the 2 bytes as a uint16_t
    return (uint16_t)(out[0] << 8 | out[1]);
}

uint8_t MemoryW25q1128jvSpi::Read_Status_Reg1(uint8_t Reg_check, uint8_t Reg_until) {
    // making sure that control pins are always high
    HAL_GPIO_WritePin(wpPort, wpPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(holdPort, holdPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

    // input operation code for reading status register 1
    uint8_t in[1] = {0x05};
    // output -- 8 bit value of the status register
    uint8_t out[1] = {0};

    // do while loop of reading status register
    // loop ends when the input Reg_check value == Reg_until
    do {
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
        if (HAL_SPI_Transmit(spi, in, 1, 10) != HAL_OK) return 0;
        if (HAL_SPI_Receive(spi, out, 1, 10) != HAL_OK) return 0;
        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    } while ((out[0] & Reg_check) == Reg_until);

    return 1;
}

uint8_t MemoryW25q1128jvSpi::Chip_Erase() {
    // making sure that control pins are always high
    HAL_GPIO_WritePin(wpPort, wpPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(holdPort, holdPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

    // instruction array for chip enable command
    uint8_t chip_enable[1] = {0x06};
    // instruction array for chip erase command
    uint8_t chip_erase[1] = {0xC7};

    // 1. enable chip with Chip_enable
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(spi, chip_enable, 1, 10) != HAL_OK) return 0;
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

    // Checks and waits until status register write enable latch is = 1
    if (MemoryW25q1128jvSpi::Read_Status_Reg1(0x02, 0) == 0) return 0;

    // 2. runs chip erase command
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(spi, chip_erase, 1, 10) != HAL_OK) return 0;
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

    // Checks and waits until status register busy bit  is = 0
    if (MemoryW25q1128jvSpi::Read_Status_Reg1(0x01, 1) == 0) return 0;

    return 1;
}

uint8_t MemoryW25q1128jvSpi::Chip_Write(afsTelemetryData data) {
    // making sure that control pins are always high
    HAL_GPIO_WritePin(wpPort, wpPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(holdPort, holdPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

    // if address variable goes over the amount of pages memory has
    // no more collected data will be written into the memory
    if (address > 0xFFFF) return 0;

    // Bundles Telemetry data struct into a 4 element array
    // Telemetry data is written into the array until all 4 elements are full
    if (Data_Bundle_Size < 3) {
        Data_Bundle[Data_Bundle_Size] = data;
        Data_Bundle_Size++;
        return 1;
    }

    // resets bundled data variable and inserts the last Telemetry data values
    Data_Bundle[3] = data;
    Data_Bundle_Size = 0;

    // Chip enable instruction command
    uint8_t chip_enable[1] = {0x06};
    // Page Program instruction command
    uint8_t Page_program[4] = {0x02, (uint8_t)(address >> 8), (uint8_t)(address), 0x00};  // instruction opcode for Page Program

    // chip enable instruction to enable chip
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(spi, chip_enable, 1, SPI_Timeout) != HAL_OK) return 0;
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

    // Checks and waits until status register write bit  is = 0
    // waits until Chip enable is done
    if (MemoryW25q1128jvSpi::Read_Status_Reg1(2, 1) == 0) return 0;

    // Write Data bundle into memory
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(spi, Page_program, 4, SPI_Timeout) != HAL_OK) return 0;
    if (HAL_SPI_Transmit(spi, (uint8_t *)Data_Bundle, 256, SPI_Timeout) != HAL_OK) return 0;
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

    // Checks and waits until status register busy bit  is = 0
    // waits until page write is done
    if (MemoryW25q1128jvSpi::Read_Status_Reg1(1, 0) == 0) return 0;

    // increments to the next page address
    address++;

    return 1;
}

MemoryW25q1128jvSpi::afsTelemetryData *MemoryW25q1128jvSpi::Chip_Read(uint32_t read_address) {
    // making sure that control pins are always high
    HAL_GPIO_WritePin(wpPort, wpPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(holdPort, holdPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

    // initalisation of input, output, and dumby telemtrydata variable for temporary writing to DataBundle
    uint8_t Read_Data[4] = {0x03, (uint8_t)(read_address >> 8), (uint8_t)(read_address), 0x00};
    uint8_t out[256] = {0};
    MemoryW25q1128jvSpi::afsTelemetryData Data;

    // Reads address page in memory
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(spi, Read_Data, 4, SPI_Timeout) != HAL_OK) return Data_Bundle;
    if (HAL_SPI_Receive(spi, out, 256, SPI_Timeout) != HAL_OK) return Data_Bundle;
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

    // Checks and waits until status register busy bit  is = 0
    // waits until page read is done
    if (MemoryW25q1128jvSpi::Read_Status_Reg1(0x01, 1) == 0) return 0;

    // organisation of Databundle from read page until Data_Bundle array
    for (int i = 0; i < 4; i++) {
        // Specifier for telemetry data
        Data.type = out[(64 * i)];
        // Timestamp for data
        Data.timestamp = ((out[(64 * i) + 4] << 24) | (out[(64 * i) + 3] << 16) | (out[(64 * i) + 2] << 8) | (out[(64 * i) + 1]));
        // What state AFS was in
        Data.state = ((out[(64 * i) + 6] << 8) | (out[(64 * i) + 5]));

        // IMU Gyroscope data
        Data.angularVelocityX = ((out[(64 * i) + 8] << 8) | (out[(64 * i) + 7]));
        Data.angularVelocityY = ((out[(64 * i) + 10] << 8) | (out[(64 * i) + 9]));
        Data.angularVelocityZ = ((out[(64 * i) + 12] << 8) | (out[(64 * i) + 11]));
        ;

        // IMU Accelerometer data
        Data.accelerationX = ((out[(64 * i) + 14] << 8) | (out[(64 * i) + 13]));
        Data.accelerationY = ((out[(64 * i) + 16] << 8) | (out[(64 * i) + 15]));
        Data.accelerationZ = ((out[(64 * i) + 18] << 8) | (out[(64 * i) + 17]));

        // IMU Magnetometer data
        Data.magneticFieldX = ((out[(64 * i) + 20] << 8) | (out[(64 * i) + 19]));
        Data.magneticFieldY = ((out[(64 * i) + 22] << 8) | (out[(64 * i) + 21]));
        Data.magneticFieldZ = ((out[(64 * i) + 24] << 8) | (out[(64 * i) + 23]));

        // altimeter data being concentated into corresponeding byes
        Data.temperature = ((out[(64 * i) + 25] << 8) | (out[(64 * i) + 24]));
        Data.altitude = ((out[(64 * i) + 29] << 24) | (out[(64 * i) + 28] << 16) | (out[(64 * i) + 27] << 8) | (out[(64 * i) + 26]));

        // ECEF Earth-Centered Earth-Fixed xyz position data
        Data.ecefPositionX = ((out[(64 * i) + 33] << 24) | (out[(64 * i) + 32] << 16) | (out[(64 * i) + 31] << 8) | (out[(64 * i) + 30]));
        Data.ecefPositionY = ((out[(64 * i) + 37] << 24) | (out[(64 * i) + 36] << 16) | (out[(64 * i) + 35] << 8) | (out[(64 * i) + 34]));
        Data.ecefPositionZ = ((out[(64 * i) + 41] << 24) | (out[(64 * i) + 40] << 16) | (out[(64 * i) + 39] << 8) | (out[(64 * i) + 38]));
        Data.ecefPositionAccuracy = ((out[(64 * i) + 45] << 24) | (out[(64 * i) + 44] << 16) | (out[(64 * i) + 43] << 8) | (out[(64 * i) + 42]));

        // GNSS Earth-Centered Earth-Fixed xyz velocity data
        Data.ecefVelocityX = ((out[(64 * i) + 50] << 24) | (out[(64 * i) + 49] << 16) | (out[(64 * i) + 48] << 8) | (out[(64 * i) + 47]));
        Data.ecefVelocityY = ((out[(64 * i) + 54] << 24) | (out[(64 * i) + 53] << 16) | (out[(64 * i) + 52] << 8) | (out[(64 * i) + 51]));
        Data.ecefVelocityZ = ((out[(64 * i) + 58] << 24) | (out[(64 * i) + 57] << 16) | (out[(64 * i) + 56] << 8) | (out[(64 * i) + 55]));
        Data.ecefVelocityAccuracy = ((out[(64 * i) + 62] << 24) | (out[(64 * i) + 61] << 16) | (out[(64 * i) + 60] << 8) | (out[(64 * i) + 59]));

        // ctc table
        Data.crc = 0x0000;

        // insertion of organised data into data bundle
        Data_Bundle[i] = Data;
    }
    return Data_Bundle;
}
