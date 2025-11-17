/*
 * ST7789.c
 *
 *  Created on: June 10, 2025
 *      Author: Loochis
 */

#include "../Inc/INA229.h"

// INITIALIZATION COMMAND LIST
// ------------------------------------------------------------------------------------

// FUNCTION IMPLEMENTEATIONS
// ------------------------------------------------------------------------------------

uint8_t INA229_ReadRegister(INA229_HandleTypeDef *ina229, uint8_t addr, uint8_t *pRead, uint8_t len) {
	// A5 A4 A3 A2 A1 A0 XX RR
	uint8_t dataToWrite = (addr << 2) | 0b1;
	uint8_t ret = 0;

	// Assert the CS low
	HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_RESET);

	ret = HAL_SPI_Transmit(ina229->spi_handle, &dataToWrite, 1, 100);
	if (ret) {
		// Release the CS
		HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_SET);
		return ERROR;
	}

	ret = HAL_SPI_Receive(ina229->spi_handle, pRead, len, 100);
	if (ret) {
		// Release the CS
		HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_SET);
		return  ERROR;
	}

	// Release the CS
	HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_SET);

	return SUCCESS;
}


uint8_t INA229_WriteRegister(INA229_HandleTypeDef *ina229, uint8_t addr, uint16_t write, uint8_t *pRead) {
	// A5 A4 A3 A2 A1 A0 XX WW DF DE DD DC DB DA D9 D8 7 D6 D5 D4 D3 D2 D1 D0
	uint8_t dataToWrite[3] = {addr << 2, write >> 8, write};
	uint8_t ret = 0;

	// Assert the CS low
	HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_RESET);
	ret = HAL_SPI_Transmit(ina229->spi_handle, &dataToWrite, 3, 100);
	if (ret) {
		// Release the CS
		HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_SET);
		return ERROR;
	}


	ret = HAL_SPI_Receive(ina229->spi_handle, pRead, 16, 100);
	if (ret) {
		// Release the CS
		HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_SET);
		return  ERROR;
	}

	// Release the CS
	HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_SET);

	return SUCCESS;
}

uint8_t INA229_Init(INA229_HandleTypeDef *ina229) {
	// SHUNT_CAL = 0x1000 at start, this is correct for R_shunt of 0.002 OHMs

	//uint8_t readVal[2];
	//INA229_ReadRegister(ina229, 0x01, readVal, 2);

	return SUCCESS;
}

uint32_t Register24_Int32(uint8_t* pData) {
	uint32_t out = 0;
	// Switch endianess from result
	out |= (uint32_t)(pData[2]);
	out |= (uint32_t)(pData[1]) << 8;
	out |= (uint32_t)(pData[0]) << 16;

	return out;
}

uint8_t INA229_Get(INA229_HandleTypeDef *ina229) {
	// Get all the power variables
	uint8_t ret = 0;

	// ----- VOLTAGE ----- //

	// Get the VBUS register value, 24 bit
	uint8_t vbus_reg[3];
	ret = INA229_ReadRegister(ina229, 0x05, vbus_reg, 3);

	// Covnert register raw to uint
	uint32_t vbus_raw = Register24_Int32(vbus_reg);
	vbus_raw >>= 4;	// Shift out the reserved bits

	// Conversion factor
	ina229->voltage = vbus_raw * 0.0001953125;

	// ----- CURRENT ----- //

	// Get the VBUS register value, 24 bit
	uint8_t cur_reg[3];
	ret = INA229_ReadRegister(ina229, 0x07, cur_reg, 3);

	// Covnert register raw to uint
	uint32_t cur_raw = Register24_Int32(cur_reg);

	int32_t cur_signed = (cur_raw << 8) | 0b111111111111;	// Pad LSBs with 1's for 2's compliment to be acurate
	cur_signed /= 4096;										// Shift out the reserved bits while respecting negativity (equiv. >> 12)

	// Conversion factor
	ina229->current = cur_signed * 0.00015625;

	if (ret) return ret;
}
