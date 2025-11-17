/*
 * STC3100.c
 *
 *  Created on: Jun 18, 2025
 *      Author: Loochis
 */

#include "STC3100.h"

// Helper
uint16_t Assemble2x8_16(uint8_t lsb, uint8_t msb) {
	uint16_t raw = 0;

	raw |= lsb;
	raw |= ((uint16_t)msb) << 8;

	return raw;
}

float VoltageToCharge(float voltage) {
	float out = 1.25 - 1.35*pow(voltage - 4.58, 2);
	if (out < 0.0) out = 0.0;
	if (out > 1.0) out = 1.0;

	return out;
}

uint8_t STC3100_ReadRegister(STC3100_HandleTypeDef *hstc, uint8_t reg, uint8_t *pData, uint8_t len) {
	// Start transfer, write reg addr as WRITE
	if (HAL_I2C_Master_Transmit(hstc->i2c_handle, ((hstc->address) << 1) | 1, &reg, 1, 10))
		return ERROR;

	// Read 2 bytes in
	if (HAL_I2C_Master_Receive(hstc->i2c_handle, ((hstc->address) << 1) | 0, pData, len, 10))
		return ERROR;

	return SUCCESS;
}

uint8_t STC3100_WriteRegister(STC3100_HandleTypeDef *hstc, uint8_t reg, uint8_t data) {
	uint8_t composite[2] = {reg, data};
	// Start transfer, write reg addr as WRITE
	if (HAL_I2C_Master_Transmit(hstc->i2c_handle, ((hstc->address) << 1) | 0, composite, 2, 10))
		return ERROR;

	return SUCCESS;
}

uint8_t STC3100_Init(STC3100_HandleTypeDef *hstc) {
	// Enable GG_RUN
	if (STC3100_WriteRegister(hstc, REG_MODE, 0b00010000))
		return ERROR;

	// un-flag PORDET, run GG_RST, set IO to Hi-Z
	if (STC3100_WriteRegister(hstc, REG_CTRL, 0b00000011))
		return ERROR;

	return SUCCESS;
}

uint8_t STC3100_Get(STC3100_HandleTypeDef *hstc) {

	uint8_t registers[12];

	// Dump the STCs registers
	if (STC3100_ReadRegister(hstc, 0, registers, 12))
		return 1;	// Read Error

	// Do some error checking
	// PORDET is flagged, battery is low
	if (registers[1] & 0b00010000)
		return 2;	// POWER LOW Error

	//TODO: Rolling average these

	int16_t charge_raw = Assemble2x8_16(registers[2], registers[3]);
	hstc->charge_delta = (float)charge_raw*0.000335; // CHARGE SINCE STARTUP [Ah]

	int16_t current_raw = Assemble2x8_16(registers[6], registers[7]) / 4;
	hstc->current = (float)current_raw*0.000585; // Current [A] (conv. fac / shunt)

	uint16_t voltage_raw = Assemble2x8_16(registers[8], registers[9]);
	hstc->voltage = (float)voltage_raw*0.00244; // Voltage [V]

	int16_t temperature_raw = Assemble2x8_16(registers[10], registers[11]);
	hstc->temperature = (float)temperature_raw*0.125; // Temp [degC]

	// Calculate other values
	if (hstc->voltage >= 4.15) {
		// CHARGING
		hstc->charge_percent = 100.0; 	// Charge level [01%]
		hstc->charge = CAPACITY_AH; 	// Charge [Ah]
		hstc->charge_time = 0.0;	// Discharge Time [sec]
	} else {
		// DISCHARGING
		hstc->charge_percent = VoltageToCharge(hstc->voltage); 	// Charge level [01%]
		hstc->charge = (hstc->charge_percent * CAPACITY_AH); 	// Charge [Ah]
		hstc->charge_time = (hstc->charge / hstc->current)*3600.0;	// Discharge Time [sec]
	}

	return SUCCESS;
}
