/*
 * STC3100.h
 *
 *  Created on: Jun 18, 2025
 *      Author: Loochis
 */

#ifndef STC3100_DRIVER_INC_STC3100_H_
#define STC3100_DRIVER_INC_STC3100_H_

#include "stm32h7xx_hal.h"

	// Define the battery capacity
	#define CAPACITY_AH 	2.0

    // Command definition
    // ------------------------------------------------------------------------------------
	#define REG_MODE				0
    #define REG_CTRL				1
    #define REG_CHARGE_LOW			2
    #define REG_CHARGE_HIGH			3
    #define REG_COUNTER_LOW			4
    #define REG_COUNTER_HIGH		5
    #define REG_CURRENT_LOW			6
    #define REG_CURRENT_HIGH		7
    #define REG_VOLTAGE_LOW			8
    #define REG_COLTAGE_HIGH		9
    #define REG_TEMPERATURE_LOW		10
    #define REG_TEMPERATURE_HIGH	11

	// STRUCTS
    // ------------------------------------------------------------------------------------
	typedef struct
	{
		I2C_HandleTypeDef 	*i2c_handle;	// ptr to I2C_HandleTypeDef which interfaces with the SSD1306

		uint8_t				address;		// address of the SSD1306, usually 0x3C

		// READ

		float 				charge_delta;

		float 				voltage;

		float 				current;

		float 				temperature;

		// COMPUTED

		float 				charge;

		uint8_t 			charge_state;

		float 				charge_percent;

		float 				charge_time;


	} STC3100_HandleTypeDef;

	// FUNCS
	// ------------------------------------------------------------------------------------

	uint8_t STC3100_ReadRegister(STC3100_HandleTypeDef *hstc, uint8_t reg, uint8_t *pData, uint8_t len);
	uint8_t STC3100_WriteRegister(STC3100_HandleTypeDef *hstc, uint8_t reg, uint8_t data);

	uint8_t STC3100_Init(STC3100_HandleTypeDef *hstc);
	uint8_t STC3100_Get(STC3100_HandleTypeDef *hstc);

#endif /* STC3100_DRIVER_INC_STC3100_H_ */
