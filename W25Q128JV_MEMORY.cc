#include <W25Q128JV_MEMORY.h>

W25Q128JV_MEMORY::W25Q128JV_MEMORY(SPI_HandleTypeDef *_spi,
								   GPIO_TypeDef *_csPort,
								   uint16_t _csPin,
								   GPIO_TypeDef *_holdPort,
								   uint16_t _holdPin,
								   GPIO_TypeDef *_wpPort,
								   uint16_t _wpPin)

								  :spi(_spi),
								   csPort(_csPort),
								   csPin(_csPin),
								   holdPort(_holdPort),
								   holdPin(_holdPin),
								   wpPort(_wpPort),
								   wpPin(_wpPin) {}


uint8_t W25Q128JV_MEMORY::Init()
{
	TelemetryData _init_Data;
	//Reads Chip ID, if chip ID is not W25Q128JV_MEMORY, then return 0
	if(DeviceID() != 0xEF17) return 0;

	// while loop to read through every memory page 
	// Result, address starts in the first empty page
	while(1)
	{		
	// Reads a page of memory
		Chip_Read(address);
		//sets first 64 byte data package into variable
		_init_Data = Data_Bundle[0];

		//compares data package type variable with 
		if(_init_Data.type != (uint8_t)(0x00)) break;
		address++;
	}
	return 1;
}

uint16_t W25Q128JV_MEMORY::DeviceID()
{
	HAL_GPIO_WritePin(wpPort, wpPin, GPIO_PIN_SET );
	HAL_GPIO_WritePin(holdPort, holdPin, GPIO_PIN_SET );
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET );

	uint8_t in[4] = {0x90,0x00,0x00,0x00};
	uint8_t out[2] = {0};

	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET );
	if (HAL_SPI_Transmit(spi, in, 4, 10) != HAL_OK) return 0;
	if (HAL_SPI_Receive(spi, out, 2, 10) != HAL_OK) return 0;
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

	return (uint16_t)(out[0]<<8 | out[1]);
}

uint8_t W25Q128JV_MEMORY::Read_Status_Reg1(uint8_t Reg_check, uint8_t Reg_until)
{
	//Reg_check: what bit to check in Status_Reg1
	//Reg_until: what Reg_check needs to be before continuing with program
	HAL_GPIO_WritePin(wpPort, wpPin, GPIO_PIN_SET );
	HAL_GPIO_WritePin(holdPort, holdPin, GPIO_PIN_SET );
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET );

	uint8_t in[1] = {0x05};
	uint8_t out[1] = {0};

	do
	{
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET );
	if (HAL_SPI_Transmit(spi, in, 1, 10) != HAL_OK) return 0;
	if (HAL_SPI_Receive(spi, out, 1, 10) != HAL_OK) return 0;
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
	}while((out[0] & Reg_check) == Reg_until);

	return 1;
}

uint8_t W25Q128JV_MEMORY::Chip_Erase()
{
	HAL_GPIO_WritePin(wpPort, wpPin, GPIO_PIN_SET );
	HAL_GPIO_WritePin(holdPort, holdPin, GPIO_PIN_SET );
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET );
	uint8_t chip_enable[1]={0x06};
	uint8_t chip_erase[1]={0xC7};

	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET );
	if (HAL_SPI_Transmit(spi, chip_enable, 1, 10) != HAL_OK) return 0;
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

	if (W25Q128JV_MEMORY::Read_Status_Reg1(0x02,0) == 0) return 0; //waits till write enable is finished

	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET );
	if (HAL_SPI_Transmit(spi, chip_erase, 1, 10) != HAL_OK) return 0;
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

	if (W25Q128JV_MEMORY::Read_Status_Reg1(0x01,1) == 0) return 0; //waits till chip erase is finished

	return 1;
}

uint8_t W25Q128JV_MEMORY::Chip_Write(TelemetryData data)
{
	HAL_GPIO_WritePin(wpPort, wpPin, GPIO_PIN_SET );
	HAL_GPIO_WritePin(holdPort, holdPin, GPIO_PIN_SET );
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET );

	if (address > 0xFFFF) return 2; //over the amount of pages that are able to write to.

	if (Data_Bundle_Size < 3)
	{
		Data_Bundle[Data_Bundle_Size] = data;
		Data_Bundle_Size++;
		return 1;
	}

	Data_Bundle[3] = data;
	Data_Bundle_Size = 0;

	uint8_t chip_enable[1]  = {0x06};									// instruction opcode for Write Enable
	uint8_t Page_program[4] = {0x02, (uint8_t)(address >> 8), (uint8_t)(address),0x00 };								// instruction opcode for Page Program

	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET );
	if (HAL_SPI_Transmit(spi, chip_enable, 1, SPI_Timeout) != HAL_OK) return 0;
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

	if (W25Q128JV_MEMORY::Read_Status_Reg1(2,1) == 0) return 0; //waits till write enable is finished

	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET );
	if (HAL_SPI_Transmit(spi, Page_program, 4,SPI_Timeout) != HAL_OK) return 0;
	if (HAL_SPI_Transmit(spi, (uint8_t *)Data_Bundle, 256, SPI_Timeout) != HAL_OK) return 0; //?? << i get its to match data types but why does it work
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

	if (W25Q128JV_MEMORY::Read_Status_Reg1(1,0) == 0) return 0; //waits till Page Program is finished

	address++;
	return 1;
}

W25Q128JV_MEMORY::TelemetryData* W25Q128JV_MEMORY::Chip_Read(uint32_t read_address)
{
	//Instruction: reads up to 256 b
	//Code initial setup
	HAL_GPIO_WritePin(wpPort, wpPin, GPIO_PIN_SET );
	HAL_GPIO_WritePin(holdPort, holdPin, GPIO_PIN_SET );
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET );

	//initalisation of input and output
	uint8_t Read_Data[4] = {0x03, (uint8_t)(read_address >> 8), (uint8_t)(read_address), 0x00};	// instruction opcode for Chip_Read
	uint8_t out[256]={0};
	W25Q128JV_MEMORY::TelemetryData Data;


	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET );
	if (HAL_SPI_Transmit(spi, Read_Data, 4, SPI_Timeout) != HAL_OK) return Data_Bundle;
	if (HAL_SPI_Receive(spi, out, 256, SPI_Timeout) != HAL_OK) return Data_Bundle;
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

	if (W25Q128JV_MEMORY::Read_Status_Reg1(0x01,1) == 0) return 0; //waits till chip erase is finished

	for(int i = 0; i < 4; i++)
	{
		Data.type = out[(64*i)];
		Data.timestamp = ((out[(64*i)+4]<<24)|(out[(64*i)+3]<<16)|(out[(64*i)+2]<<8)|(out[(64*i)+1]));
		Data.state = ((out[(64*i)+6]<<8)|(out[(64*i)+5]));

		//bad down here
		Data.altimeterTemperature = ((out[(64*i)+25]<<8) | (out[(64*i)+24]));
		Data.altimeterAltitude = 	((out[(64*i)+29]<<24) | (out[(64*i)+28]<<16) | (out[(64*i)+27]<<8) | (out[(64*i)+26]));

		Data_Bundle[i] = Data;
	}
	return Data_Bundle;
}
