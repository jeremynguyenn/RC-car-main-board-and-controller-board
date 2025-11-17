/*
 * ST7789.c
 *
 *  Created on: May 17, 2025
 *      Author: Loochis
 */

#include "ST7789.h"

// INITIALIZATION COMMAND LIST
// ------------------------------------------------------------------------------------
// Im gonna keep it a buck, I have no idea what any of this does
// See: https://www.waveshare.com/wiki/2inch_LCD_Module#Software_description
// See: https://www.mouser.com/datasheet/2/744/ST7789VW-2320339.pdf
const uint8_t ST7789_INITCMDS[] = {
	19,                                                       // number of initializers
	1,  0x36, 0x00,
	1,  0x3A, 0x05,
	0,  0x21,
	4,  0x2A, 0x00, 0x00, 0x01, 0x3F,
	4,  0x2B, 0x00, 0x00, 0x00, 0xEF,
	5,  0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33,
	1,  0xB7, 0x35,
	1,  0xBB, 0x1F,
	1,  0xC0, 0x2C,
	1,  0xC2, 0x01,
	1,  0xC3, 0x12,
	1,  0xC4, 0x20,
	1,  0xC6, 0x0F,
	2,  0xD0, 0xA4, 0xA1,
	14, 0xE0, 0xD0, 0x08, 0x11, 0x08, 0x0C,
	          0x15, 0x39, 0x33, 0x50, 0x36,
			  0x13, 0x14, 0x29, 0x2D,
	14, 0xE1, 0xD0, 0x08, 0x10, 0x08, 0x06,
	          0x06, 0x39, 0x44, 0x51, 0x0B,
			  0x16, 0x14, 0x2F, 0x31,
	0,  0x21,
	0,  0x11,
	0,  0x29
};

// FONT
// ------------------------------------------------------------------------------------
const char TXT_NOSIGNAL[] = {
	    0x00, 0b11111110, 0b00001100, 0b00010000, 0b01100000, 0b11111110, 0x00, // 0 - N
	    0x00, 0b01111100, 0b10000010, 0b10000010, 0b10000010, 0b01111100, 0x00, // 1 - O
	    0x00, 0x00,       0x00,       0x00,       0x00,       0x00,       0x00, // 2 - SP
		0x00, 0b01001100, 0b10010010, 0b10010010, 0b10010010, 0b01100100, 0x00, // 3 - S
		0x00, 0b10000010, 0b10000010, 0b11111110, 0b10000010, 0b10000010, 0x00, // 4 - I
		0x00, 0b01111110, 0b10000010, 0b10000010, 0b10010010, 0b01110100, 0x00, // 5 - G
		0x00, 0b11111110, 0b00001100, 0b00010000, 0b01100000, 0b11111110, 0x00, // 6 - N
		0x00, 0b11111100, 0b00010010, 0b00010010, 0b00010010, 0b11111110, 0x00, // 7 - A
		0x00, 0b11111110, 0b10000000, 0b10000000, 0b10000000, 0b10000000, 0x00  // 8 - L
};

const char TXT_MS[] = {
		0x00, 0b11111110, 0b00000010, 0b00011100, 0b00000010, 0b11111110, 0x00, // 0 - M
		0x00, 0b01001100, 0b10010010, 0b10010010, 0b10010010, 0b01100100, 0x00, // 1 - S
		0x00, 0x00,       0x00,       0x00,       0x00,       0x00,       0x00  // 2 - SP
};

const char TXT_NUM[] = {
	    0x00, 0b01111100, 0b11100010, 0b10010010, 0b10001110, 0b01111100, 0x00, // 0
	    0x00, 0b10000010, 0b10000010, 0b11111110, 0b10000000, 0b10000000, 0x00, // 1
	    0x00, 0b11100100, 0b10010010, 0b10010010, 0b10010010, 0b10001100, 0x00, // 2
	    0x00, 0b01000100, 0b10000010, 0b10010010, 0b10010010, 0b01101100, 0x00, // 3
	    0x00, 0b00011110, 0b00010000, 0b00010000, 0b00010000, 0b11111110, 0x00, // 4
	    0x00, 0b01001110, 0b10010010, 0b10010010, 0b10010010, 0b01100010, 0x00, // 5
	    0x00, 0b01111100, 0b10010010, 0b10010010, 0b10010010, 0b01100100, 0x00, // 6
	    0x00, 0b10000010, 0b01100010, 0b00010010, 0b00001110, 0b00000010, 0x00, // 7
	    0x00, 0b01101100, 0b10010010, 0b10010010, 0b10010010, 0b01101100, 0x00, // 8
	    0x00, 0b01001100, 0b10010010, 0b10010010, 0b10010010, 0b01111100, 0x00  // 9
};

// FUNCTION IMPLEMENTEATIONS
// ------------------------------------------------------------------------------------

// Writes a single command byte to the LCD
uint8_t ST7789_SendByte_Command(ST7789_HandleTypeDef *hst7789, uint8_t command) {
	HAL_GPIO_WritePin(hst7789->dc_gpio_handle, hst7789->dc_gpio_pin, GPIO_PIN_RESET);	// assert DC LO (~CMD)

	// Write the data
	if (HAL_SPI_Transmit(hst7789->spi_handle, &command, 1, 500))
		return ERROR;
	return SUCCESS;
}

uint8_t ST7789_SendByte_Data(ST7789_HandleTypeDef *hst7789, uint8_t data) {
	HAL_GPIO_WritePin(hst7789->dc_gpio_handle, hst7789->dc_gpio_pin, GPIO_PIN_SET);		// assert DC HI (DATA)

	// Write the data
	if (HAL_SPI_Transmit(hst7789->spi_handle, &data, 1, 500))
		return ERROR;
	return SUCCESS;
}

uint8_t ST7789_SendWord_Data(ST7789_HandleTypeDef *hst7789, uint16_t data) {
	HAL_GPIO_WritePin(hst7789->dc_gpio_handle, hst7789->dc_gpio_pin, GPIO_PIN_SET);		// assert DC HI (DATA)

	// Write the data
	if (HAL_SPI_Transmit(hst7789->spi_handle, (uint8_t*)(&data), 2, 500))
		return ERROR;
	return SUCCESS;
}

uint8_t ST7789_Init(ST7789_HandleTypeDef *hst7789) {

	hst7789->update_sequence = 2;

	// Wake up the SPI line
	uint8_t dummy = 0x00;
	HAL_SPI_Transmit_DMA(hst7789->spi_handle, &dummy, 1);
	HAL_Delay(10);

	// Some control variables
	uint16_t n_commands = ST7789_INITCMDS[0];
	uint16_t n_arguments;
	uint16_t cmd_idx = 1;

	// Read the init sequence
	while (n_commands--) {
		// Get no. of Args
	    n_arguments = ST7789_INITCMDS[cmd_idx];
	    cmd_idx++;

	    // Send initial command
	    if (ST7789_SendByte_Command(hst7789, ST7789_INITCMDS[cmd_idx])) return cmd_idx;
	    cmd_idx++;

	    // Send argumemts
	    while (n_arguments--) {
			if (ST7789_SendByte_Data(hst7789, ST7789_INITCMDS[cmd_idx])) return cmd_idx;
				cmd_idx++;
	    }
	}
	return SUCCESS;
}

// Set the cursor
void ST7789_SetCursor(ST7789_HandleTypeDef *hst7789, uint16_t x, uint16_t y) {
	ST7789_SendByte_Command(hst7789, 0x2a);
	ST7789_SendByte_Data(hst7789, x >> 8);
	ST7789_SendByte_Data(hst7789, x);
	ST7789_SendByte_Data(hst7789, x >> 8);
	ST7789_SendByte_Data(hst7789, x);

	ST7789_SendByte_Command(hst7789, 0x2b);
	ST7789_SendByte_Data(hst7789, y >> 8);
	ST7789_SendByte_Data(hst7789, y);
	ST7789_SendByte_Data(hst7789, y >> 8);
	ST7789_SendByte_Data(hst7789, y);

	ST7789_SendByte_Command(hst7789, 0x2C);
}

// Sets the "window"?
void ST7789_SetWindow(ST7789_HandleTypeDef *hst7789, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t  yEnd) {
	ST7789_SendByte_Command(hst7789, 0x2a);
	ST7789_SendByte_Data(hst7789, xStart >>8);
	ST7789_SendByte_Data(hst7789, xStart & 0xff);
	ST7789_SendByte_Data(hst7789, (xEnd - 1) >> 8);
	ST7789_SendByte_Data(hst7789, (xEnd - 1) & 0xff);

	ST7789_SendByte_Command(hst7789, 0x2b);
	ST7789_SendByte_Data(hst7789, yStart >>8);
	ST7789_SendByte_Data(hst7789, yStart & 0xff);
	ST7789_SendByte_Data(hst7789, (yEnd - 1) >> 8);
	ST7789_SendByte_Data(hst7789, (yEnd - 1) & 0xff);

	ST7789_SendByte_Command(hst7789, 0x2C);
}

uint8_t ST7789_UpdateSector(ST7789_HandleTypeDef *hst7789, uint8_t screen_section) {
	// Check for bounds/busy
	if (hst7789->spi_state == 1) return ERROR;
	if (screen_section > 2) return ERROR;

	// Flag as busy
	hst7789->spi_state = 1;

	// Set the window based on the vram offset
	ST7789_SetWindow(hst7789, 0, (screen_section*0xEA60)/(LCD_WIDTH*2), LCD_WIDTH, LCD_HEIGHT);

	HAL_GPIO_WritePin(hst7789->dc_gpio_handle, hst7789->dc_gpio_pin, GPIO_PIN_SET);		// assert DC HI (~CMD)


	if (screen_section != 2) {
		if (HAL_SPI_Transmit_DMA(hst7789->spi_handle, hst7789->vram + screen_section*0xEA60, 0xEA60))
			return ERROR;
	} else {
		if (HAL_SPI_Transmit_DMA(hst7789->spi_handle, hst7789->vram + screen_section*0xEA60, 0x8340))
			return ERROR;
	}
//	return SUCCESS;
	//HAL_SPI_Transmit(hst7789->spi_handle, hst7789->vram, 0xEA60, 500);
	return SUCCESS;
}

uint8_t ST7789_UpdateAutomatic(ST7789_HandleTypeDef *hst7789) {
	// Reset the sector counter
	hst7789->update_sequence = 0;

	// perform a screen update
	if (ST7789_UpdateSector(hst7789, hst7789->update_sequence)) return ERROR;

	return SUCCESS;
}


uint8_t ST7789_Clear(ST7789_HandleTypeDef *hst7789) {
	// fill VRAM with black
	memset(hst7789->vram, 0x00, LCD_WIDTH*LCD_HEIGHT*2);
	return SUCCESS;
}

uint8_t ST7789_Draw_NOSIG(ST7789_HandleTypeDef *hst7789) {

	// Clear the area
	uint32_t cursor = LCD_WIDTH*(LCD_HEIGHT - 7*FONTSCALE_NOSIGNAL)*2 + (LCD_WIDTH - 7*FONTSCALE_NOSIGNAL);
	for (uint32_t x = 0; x < 66*FONTSCALE_NOSIGNAL; x++) {
		memset(hst7789->vram + cursor - x*LCD_WIDTH*2, 0x00, 24*FONTSCALE_NOSIGNAL);
	}

	// Draw the NO SIGNAL symbol
		cursor = LCD_WIDTH*(LCD_HEIGHT - 9*FONTSCALE_NOSIGNAL)*2 + (LCD_WIDTH - 4*FONTSCALE_NOSIGNAL);
		for (uint8_t c = 0; c < 9; c++) {		// Loop chars
			for (uint8_t l = 0; l < 7; l++) {	// Loop lines
				uint8_t line_byte = TXT_NOSIGNAL[c*7 + l];

				for (uint8_t b = 0; b < 8; b++) {		// Loop bits
					if ((line_byte >> b) & 0x01) {	// Check if bit is 1
						for (uint8_t y = 0; y < FONTSCALE_NOSIGNAL; y++) {
							for (uint8_t x = 0; x < FONTSCALE_NOSIGNAL; x++) {
								hst7789->vram[(x*LCD_WIDTH + y + b*FONTSCALE_NOSIGNAL)*2 + cursor    ] = 0xFF;
								hst7789->vram[(x*LCD_WIDTH + y + b*FONTSCALE_NOSIGNAL)*2 + cursor + 1] = 0xFF;
							}
						}
					}
				}
				cursor -= LCD_WIDTH*FONTSCALE_NOSIGNAL*2;
			}
		}

		return SUCCESS;
}

uint8_t ST7789_Draw_DATA(ST7789_HandleTypeDef *hst7789, uint32_t frametime_ms) {

	uint16_t ms = frametime_ms;
	if (ms > 999)
		ms = 999;

	uint8_t digits[3] = {
			(ms / 100) % 10,
			(ms / 10)  % 10,
			 ms        % 10
	};

	// Clear the corner
	uint32_t cursor = (LCD_WIDTH*(LCD_HEIGHT-2) + 2)*2;
	for (uint32_t x = 0; x < 44*FONTSCALE_FRAMETIME; x++) {
		memset(hst7789->vram + cursor - x*LCD_WIDTH*2, 0x00, 24*FONTSCALE_FRAMETIME);
	}


	// Draw the MS symbol
	cursor = (LCD_WIDTH*(LCD_HEIGHT-2) + 2)*2;
	for (uint8_t c = 0; c < 3; c++) {		// Loop chars
		for (uint8_t l = 0; l < 7; l++) {	// Loop lines
			uint8_t line_byte = TXT_MS[c*7 + l];

			for (uint8_t b = 0; b < 8; b++) {		// Loop bits
				if ((line_byte >> b) & 0x01) {	// Check if bit is 1
					for (uint8_t y = 0; y < FONTSCALE_FRAMETIME; y++) {
						for (uint8_t x = 0; x < FONTSCALE_FRAMETIME; x++) {
							hst7789->vram[(x*LCD_WIDTH + y + b*FONTSCALE_FRAMETIME)*2 + cursor    ] = 0xFF;
							hst7789->vram[(x*LCD_WIDTH + y + b*FONTSCALE_FRAMETIME)*2 + cursor + 1] = 0xFF;
						}
					}
				}
			}
			cursor -= LCD_WIDTH*FONTSCALE_FRAMETIME*2;
		}
	}

	// Draw the Digits
	for (uint8_t c = 0; c < 3; c++) {		// Loop digits
			for (uint8_t l = 0; l < 7; l++) {	// Loop lines
				uint8_t line_byte = TXT_NUM[digits[c]*7 + l];

				for (uint8_t b = 0; b < 8; b++) {		// Loop bits
					if ((line_byte >> b) & 0x01) {	// Check if bit is 1
						for (uint8_t y = 0; y < FONTSCALE_FRAMETIME; y++) {
							for (uint8_t x = 0; x < FONTSCALE_FRAMETIME; x++) {
								hst7789->vram[(x*LCD_WIDTH + y + b*FONTSCALE_FRAMETIME)*2 + cursor    ] = 0xFF;
								hst7789->vram[(x*LCD_WIDTH + y + b*FONTSCALE_FRAMETIME)*2 + cursor + 1] = 0xFF;
							}
						}
					}
				}
				cursor -= LCD_WIDTH*FONTSCALE_FRAMETIME*2;
			}
		}

	return SUCCESS;
}

void ST7789_DMATransmitCplt(ST7789_HandleTypeDef *hst7789) {
	// Flag idle
	hst7789->spi_state = 0;

	// Check the state of the update sequence
	if (hst7789->update_sequence < 2) {
		// Immediately start the next update in the sequence
		hst7789->update_sequence++;
		ST7789_UpdateSector(hst7789, hst7789->update_sequence);
	}
}
