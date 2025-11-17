/*
 * ST7789.h
 *
 *  Created on: May 17, 2025
 *      Author: Loochis
 */

#ifndef INA229_DRIVER_INC_INA229_H_
#define INA229_DRIVER_INC_INA229_H_

#include "stm32h7xx_hal.h"

// STRUCTS
// ------------------------------------------------------------------------------------
typedef struct
{
	SPI_HandleTypeDef 	*spi_handle;	// ptr to I2C_HandleTypeDef which interfaces with the ST7789

	GPIO_TypeDef		*cs_gpio_handle;// ptr to GPIO handle of CS line

	uint16_t			cs_gpio_pin;	// GPIO pin no. of CS line

	float 				voltage;

	float 				current;

	float 				temperature;

} INA229_HandleTypeDef;

	// FUNCS
	// ------------------------------------------------------------------------------------

//	uint8_t ST7789_SendByte_Command(ST7789_HandleTypeDef *hst7789, uint8_t command);
//	uint8_t ST7789_SendByte_Data(ST7789_HandleTypeDef *hst7789, uint8_t data);
//	uint8_t ST7789_SendWord_Data(ST7789_HandleTypeDef *hst7789, uint16_t data);

uint8_t INA229_ReadRegister(INA229_HandleTypeDef *ina229, uint8_t addr, uint8_t *pRead, uint8_t len);
uint8_t INA229_WriteRegister(INA229_HandleTypeDef *ina229, uint8_t addr, uint16_t write, uint8_t *pRead);

uint8_t INA229_Init(INA229_HandleTypeDef *ina229);
uint8_t INA229_Get(INA229_HandleTypeDef *ina229);


#endif /* INA229_DRIVER_INC_INA229_H_ */
