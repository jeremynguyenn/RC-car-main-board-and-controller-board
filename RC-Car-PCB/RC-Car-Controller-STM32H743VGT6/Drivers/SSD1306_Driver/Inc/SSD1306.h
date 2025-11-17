/*
 * SSD1306.h
 *
 *  Created on: May 15, 2025
 *      Author: Loochis
 */

#ifndef SSD1306_DRIVER_INC_SSD1306_H_
#define SSD1306_DRIVER_INC_SSD1306_H_

#include "stm32h7xx_hal.h"

    // Command definition
    // ------------------------------------------------------------------------------------
    #define COMMAND           0x80  // Continuation bit=1, D/C=0; 1000 0000
    #define COMMAND_STREAM    0x00  // Continuation bit=0, D/C=0; 0000 0000
    #define DATA              0xC0  // Continuation bit=1, D/C=1; 1100 0000
    #define DATA_STREAM       0x40  // Continuation bit=0, D/C=1; 0100 0000

    #define SET_MUX_RATIO     0xA8
    #define DISPLAY_OFFSET    0xD3
    #define DISPLAY_ON        0xAF
    #define DISPLAY_OFF       0xAE
    #define DIS_ENT_DISP_ON   0xA4
    #define DIS_IGNORE_RAM    0xA5
    #define DIS_NORMAL        0xA6
    #define DIS_INVERSE       0xA7
    #define DEACT_SCROLL      0x2E
    #define ACTIVE_SCROLL     0x2F
    #define SET_START_LINE    0x40
    #define MEMORY_ADDR_MODE  0x20
    #define SET_COLUMN_ADDR   0x21
    #define SET_PAGE_ADDR     0x22
    #define SEG_REMAP         0xA0
    #define SEG_REMAP_OP      0xA1
    #define COM_SCAN_DIR      0xC0
    #define COM_SCAN_DIR_OP   0xC8
    #define COM_PIN_CONF      0xDA
    #define SET_CONTRAST      0x81
    #define SET_OSC_FREQ      0xD5
    #define SET_CHAR_REG      0x8D
    #define SET_PRECHARGE     0xD9
    #define VCOM_DESELECT     0xDB

    // Clear Color
    // ------------------------------------------------------------------------------------
    #define CLEAR_COLOR               0x00

    // Init Status
    // ------------------------------------------------------------------------------------
    #define INIT_STATUS               0xFF

    // AREA definition
    // ------------------------------------------------------------------------------------
    #define START_PAGE_ADDR           0
    #define END_PAGE_ADDR             7     // 7 for 128x64, 3 for 128x32 version
    #define START_COLUMN_ADDR         0
    #define END_COLUMN_ADDR           127
    #define RAM_X_END                 END_COLUMN_ADDR + 1
    #define RAM_Y_END                 END_PAGE_ADDR + 1

    #define CACHE_SIZE_MEM            (1 + END_PAGE_ADDR) * (1 + END_COLUMN_ADDR)

    #define MAX_X                     END_COLUMN_ADDR
    #define MAX_Y                     (END_PAGE_ADDR + 1) * 8

	// STRUCTS
    // ------------------------------------------------------------------------------------
	typedef struct
	{
		I2C_HandleTypeDef 	*i2c_handle;	// ptr to I2C_HandleTypeDef which interfaces with the SSD1306

		uint8_t				address;		// address of the SSD1306, usually 0x3C

		uint8_t				*vram_full;		// ptr to the MCU side copy of SSD1306 VRAM including the data type byte

		uint8_t				*vram;			// ptr to the MCU side copy of SSD1306 VRAM

		uint16_t			str_cursor;		// position that text commands will start writing from

		uint8_t				draw_inverted;

		uint8_t				draw_scale;

	} SSD1306_HandleTypeDef;

	// FUNCS
	// ------------------------------------------------------------------------------------

	uint8_t SSD1306_SendCommand(SSD1306_HandleTypeDef *hssd, uint8_t command);
	uint8_t SSD1306_Init(SSD1306_HandleTypeDef *hssd);
	uint8_t SSD1306_Clear(SSD1306_HandleTypeDef *hssd);
	uint8_t SSD1306_Update(SSD1306_HandleTypeDef *hssd);

	uint8_t SSD1306_DrawChar(SSD1306_HandleTypeDef *hssd, char ch);
	uint8_t SSD1306_DrawString(SSD1306_HandleTypeDef *hssd, char *str, uint8_t length);

#endif /* SSD1306_DRIVER_INC_SSD1306_H_ */
