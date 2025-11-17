/*
 * SSD1306.c
 *
 *  Created on: May 15, 2025
 *      Author: Loochis
 */

#include "SSD1306.h"

// INITIALIZATION COMMAND LIST
// ------------------------------------------------------------------------------------
const uint8_t SSD1306_INITCMDS[] = {
	18,                                                       // number of initializers
	0, DISPLAY_OFF,                                           // 0xAE = Set Display OFF
	1, SET_MUX_RATIO, 63,                                     // 0xA8 - 64MUX for 128 x 64 version
	                                                          	  //      - 32MUX for 128 x 32 version
	1, MEMORY_ADDR_MODE, 0x00,                                // 0x20 = Set Memory Addressing Mode
	                                                          	  // 0x00 - Horizontal Addressing Mode
	                                                          	  // 0x01 - Vertical Addressing Mode
	                                                          	  // 0x02 - Page Addressing Mode (RESET)
	2, SET_COLUMN_ADDR, START_COLUMN_ADDR, END_COLUMN_ADDR,   // 0x21 = Set Column Address, 0 - 127
	2, SET_PAGE_ADDR, START_PAGE_ADDR, END_PAGE_ADDR,         // 0x22 = Set Page Address, 0 - 7
	0, SET_START_LINE,                                        // 0x40
	1, DISPLAY_OFFSET, 0x80,                                  // 0xD3
	0, SEG_REMAP_OP,                                          // 0xA0 / remap 0xA1
	0, COM_SCAN_DIR_OP,                                       // 0xC0 / remap 0xC8
	1, COM_PIN_CONF, 0x12,                                    // 0xDA, 0x12 - Disable COM Left/Right remap, Alternative COM pin configuration
	                                                          	  //       0x12 - for 128 x 64 version
	                                                          	  //       0x02 - for 128 x 32 version
	1, SET_CONTRAST, 0x7F,                                    // 0x81, 0x7F - reset value (max 0xFF)
	0, DIS_ENT_DISP_ON,                                       // 0xA4
	0, DIS_NORMAL,                                            // 0xA6
	1, SET_OSC_FREQ, 0x80,                                    // 0xD5, 0x80 => D=1; DCLK = Fosc / D <=> DCLK = Fosc
	1, SET_PRECHARGE, 0xc2,                                   // 0xD9, higher value less blinking
	                                                          	  // 0xC2, 1st phase = 2 DCLK,  2nd phase = 13 DCLK
	1, VCOM_DESELECT, 0x20,                                   // Set V COMH Deselect, reset value 0x22 = 0,77xUcc
	1, SET_CHAR_REG, 0x14,                                    // 0x8D, Enable charge pump during display on
	0, DISPLAY_ON                                             // 0xAF = Set Display ON
};

// FONT
// ------------------------------------------------------------------------------------
const char ALPHNUM[] = {
//  LEN   LS    --------------------------------->    RS
    0x06, 0x00, 0x00,       0x00,       0x00,       0x00,       0x00,       0x00, // ------ 32 - SP
    0x02, 0x00, 0b10111110, 0x00,       0x00,       0x00,       0x00,       0x00, // ------ 33 - !
    0x04, 0x00, 0b00000110, 0x00,       0b00000110, 0x00,       0x00,       0x00, // ------ 34 - "
    0x06, 0x00, 0b00101000, 0b11111110, 0b00101000, 0b11111110, 0b00101000, 0x00, // ------ 35 - #
    0x06, 0x00, 0b01001000, 0b01010100, 0b11111110, 0b01010100, 0b00100100, 0x00, // ------ 36 - $
    0x06, 0x00, 0b10000100, 0b01101010, 0b01010100, 0b10101100, 0b01000010, 0x00, // ------ 37 - %
    0x06, 0x00, 0b01101100, 0b10010010, 0b10110010, 0b01001100, 0b10100000, 0x00, // ------ 38 - &
    0x02, 0x00, 0b00000110, 0x00,       0x00,       0x00,       0x00,       0x00, // ------ 39 - '
    0x04, 0x00, 0x00,       0b01111100, 0b10000010, 0x00,       0x00,       0x00, // ------ 40 - (
    0x04, 0x00, 0b10000010, 0b01111100, 0x00,       0x00,       0x00,       0x00, // ------ 41 - )
    0x04, 0x00, 0b00001010, 0b00000100, 0b00001010, 0x00,       0x00,       0x00, // ------ 42 - *
    0x06, 0x00, 0b00010000, 0b00010000, 0b01111100, 0b00010000, 0b00010000, 0x00, // ------ 43 - +
    0x02, 0x00, 0b11000000, 0x00,       0x00,       0x00,       0x00,       0x00, // ------ 44 - ,
    0x06, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0b00010000, 0x00, // ------ 45 - -
    0x03, 0x00, 0b11000000, 0b11000000, 0x00,       0x00,       0x00,       0x00, // ------ 46 - .
    0x06, 0x00, 0b10000000, 0b01100000, 0b00010000, 0b00001100, 0b00000010, 0x00, // ------ 47 - /
    0x06, 0x00, 0b01111100, 0b11100010, 0b10010010, 0b10001110, 0b01111100, 0x00, // ------ 48 - 0
    0x06, 0x00, 0b10000010, 0b10000010, 0b11111110, 0b10000000, 0b10000000, 0x00, // ------ 49 - 1
    0x06, 0x00, 0b11100100, 0b10010010, 0b10010010, 0b10010010, 0b10001100, 0x00, // ------ 50 - 2
    0x06, 0x00, 0b01000100, 0b10000010, 0b10010010, 0b10010010, 0b01101100, 0x00, // ------ 51 - 3
    0x06, 0x00, 0b00011110, 0b00010000, 0b00010000, 0b00010000, 0b11111110, 0x00, // ------ 52 - 4
    0x06, 0x00, 0b01001110, 0b10010010, 0b10010010, 0b10010010, 0b01100010, 0x00, // ------ 53 - 5
    0x06, 0x00, 0b01111100, 0b10010010, 0b10010010, 0b10010010, 0b01100100, 0x00, // ------ 54 - 6
    0x06, 0x00, 0b10000010, 0b01100010, 0b00010010, 0b00001110, 0b00000010, 0x00, // ------ 55 - 7
    0x06, 0x00, 0b01101100, 0b10010010, 0b10010010, 0b10010010, 0b01101100, 0x00, // ------ 56 - 8
    0x06, 0x00, 0b01001100, 0b10010010, 0b10010010, 0b10010010, 0b01111100, 0x00, // ------ 57 - 9
    0x03, 0x00, 0b11000110, 0b11000110, 0x00,       0x00,       0x00,       0x00, // ------ 58 - :
    0x03, 0x00, 0b11000110, 0b00000110, 0x00,       0x00,       0x00,       0x00, // ------ 59 - ;
    0x06, 0x00, 0b00010000, 0b00101000, 0b01000100, 0b10000010, 0x00,       0x00, // ------ 60 - <
    0x06, 0x00, 0b00101000, 0b00101000, 0b00101000, 0b00101000, 0b00101000, 0x00, // ------ 61 - =
    0x06, 0x00, 0b10000010, 0b01000100, 0b00101000, 0b00010000, 0x00,       0x00, // ------ 62 - >
    0x06, 0x00, 0b00000100, 0b00000010, 0b10110010, 0b00010010, 0b00001100, 0x00, // ------ 63 - ?
    0x07, 0x00, 0b00111000, 0b01010100, 0b10101010, 0b10111010, 0b10100100, 0b00011000, //  64 - @

    0x06, 0x00, 0b11111100, 0b00010010, 0b00010010, 0b00010010, 0b11111110, 0x00, // 65 - A
    0x06, 0x00, 0b11111110, 0b10010010, 0b10010010, 0b10010010, 0b01101100, 0x00, // 66 - B
    0x06, 0x00, 0b01111100, 0b10000010, 0b10000010, 0b10000010, 0b01000100, 0x00, // 67 - C
    0x06, 0x00, 0b11111110, 0b10000010, 0b10000010, 0b10000010, 0b01111100, 0x00, // 68 - D
    0x06, 0x00, 0b11111110, 0b10010010, 0b10010010, 0b10010010, 0b10000010, 0x00, // 69 - E
    0x06, 0x00, 0b11111110, 0b00010010, 0b00010010, 0b00010010, 0b00000010, 0x00, // 70 - F
    0x06, 0x00, 0b01111110, 0b10000010, 0b10000010, 0b10010010, 0b01110100, 0x00, // 71 - G
    0x06, 0x00, 0b11111110, 0b00010000, 0b00010000, 0b00010000, 0b11111110, 0x00, // 72 - H
    0x06, 0x00, 0b10000010, 0b10000010, 0b11111110, 0b10000010, 0b10000010, 0x00, // 73 - I
    0x06, 0x00, 0b01000000, 0b10000000, 0b10000010, 0b10000010, 0b01111110, 0x00, // 74 - J
    0x06, 0x00, 0b11111110, 0b00010000, 0b00010000, 0b00101000, 0b11000110, 0x00, // 75 - K
    0x06, 0x00, 0b11111110, 0b10000000, 0b10000000, 0b10000000, 0b10000000, 0x00, // 76 - L
    0x06, 0x00, 0b11111110, 0b00000010, 0b00011100, 0b00000010, 0b11111110, 0x00, // 77 - M
    0x06, 0x00, 0b11111110, 0b00001100, 0b00010000, 0b01100000, 0b11111110, 0x00, // 78 - N
    0x06, 0x00, 0b01111100, 0b10000010, 0b10000010, 0b10000010, 0b01111100, 0x00, // 79 - O
    0x06, 0x00, 0b11111110, 0b00010010, 0b00010010, 0b00010010, 0b00001100, 0x00, // 80 - P
    0x06, 0x00, 0b01111100, 0b10000010, 0b10100010, 0b01000010, 0b10111100, 0x00, // 81 - Q
    0x06, 0x00, 0b11111110, 0b00010010, 0b00010010, 0b00010010, 0b11101100, 0x00, // 82 - R
    0x06, 0x00, 0b01001100, 0b10010010, 0b10010010, 0b10010010, 0b01100100, 0x00, // 83 - S
    0x06, 0x00, 0b00000010, 0b00000010, 0b11111110, 0b00000010, 0b00000010, 0x00, // 84 - T
    0x06, 0x00, 0b01111110, 0b10000000, 0b10000000, 0b10000000, 0b01111110, 0x00, // 85 - U
    0x06, 0x00, 0b00011110, 0b01100000, 0b10000000, 0b01100000, 0b00011110, 0x00, // 86 - V
    0x06, 0x00, 0b11111110, 0b10000000, 0b01110000, 0b10000000, 0b11111110, 0x00, // 87 - W
    0x06, 0x00, 0b10000010, 0b01101100, 0b00010000, 0b01101100, 0b10000010, 0x00, // 88 - X
    0x06, 0x00, 0b00000010, 0b00001100, 0b11110000, 0b00001100, 0b00000010, 0x00, // 89 - Y
    0x06, 0x00, 0b10000010, 0b11100010, 0b10010010, 0b10001110, 0b10000010, 0x00, // 90 - Z

    0x04, 0x00, 0x00,       0b11111110, 0b10000010, 0x00,       0x00,       0x00, // ------ 91 - [
    0x06, 0x00, 0b00000010, 0b00001100, 0b00010000, 0b01100000, 0b10000000, 0x00, // ------ 92 -
    0x04, 0x00, 0b10000010, 0b11111110, 0x00,       0x00,       0x00,       0x00, // ------ 93 - ]
    0x06, 0x00, 0b00001000, 0b00000100, 0b00000010, 0b00000100, 0b00001000, 0x00, // ------ 94 - ^
    0x06, 0x00, 0b10000000, 0b10000000, 0b10000000, 0b10000000, 0b10000000, 0x00, // ------ 95 - _
    0x03, 0x00, 0b00000110, 0b00000100,       0x00,       0x00,       0x00, 0x00, // ------ 96 - `

                                                                                  // MAP (97-122 -> 65-90)

    0x04, 0x00, 0b00010000, 0b01111100, 0b10000010, 0x00,       0x00,       0x00, // ------ 123 - {
    0x02, 0x00, 0b11111110, 0x00,       0x00,       0x00,       0x00,       0x00, // ------ 124 - |
    0x04, 0x00, 0b10000010, 0b01111100, 0b00010000, 0x00,       0x00,       0x00, // ------ 125 - }
    0x06, 0x00, 0b00010000, 0b00001000, 0b00010000, 0b00100000, 0b00010000, 0x00, // ------ 126 - ~

	0x06, 0x00, 0x00,       0x00,       0x00,       0x00,       0x00,       0x00, // ------ 127 - DEL

	0x06, 0b10001000, 0b00100010, 0b10001000, 0b00100010, 0b10001000, 0b00100010, 0b10001000, // ------ 128 - \x80 (░)
	0x06, 0b10101010, 0b01010101, 0b10101010, 0b01010101, 0b10101010, 0b01010101, 0b10101010, // ------ 129 - \x81 (▒)
	0x06, 0b01110111, 0b11011101, 0b01110111, 0b11011101, 0b01110111, 0b11011101, 0b01110111, // ------ 130 - \x82 (▓)
	0x06, 0xFF,       0xFF,       0xFF,       0xFF,       0xFF,       0xFF,       0xFF,       // ------ 131 - \x83 (█)
	0x06, 0x00,       0xFF,       0x00,       0xFF,       0x00,       0xFF,       0x00        // ------ 131 - \x84 (|||)
};

// FUNCTION IMPLEMENTEATIONS
// ------------------------------------------------------------------------------------
uint8_t SSD1306_SendCommand(SSD1306_HandleTypeDef *hssd, uint8_t command) {
	uint8_t composite[2] = {COMMAND, command};
	if (HAL_I2C_Master_Transmit(hssd->i2c_handle, (hssd->address) << 1, composite, 2, 100))
		return ERROR;
	return SUCCESS;
}


uint8_t SSD1306_Init(SSD1306_HandleTypeDef *hssd) {
	// Set the cursor
	hssd->str_cursor = 0;
	hssd->vram = hssd->vram_full + 1;
	// Some control variables
	uint16_t n_commands = SSD1306_INITCMDS[0];
	uint16_t n_arguments;
	uint16_t cmd_idx = 1;

	// Read the init sequence
	while (n_commands--) {
	        // Get no. of Args
	        n_arguments = SSD1306_INITCMDS[cmd_idx];
	        cmd_idx++;
	        // Send initial command
	        if (SSD1306_SendCommand(hssd, SSD1306_INITCMDS[cmd_idx])) return cmd_idx;
	        cmd_idx++;

	        // Send argumemts
	        while (n_arguments--) {
	            if (SSD1306_SendCommand(hssd, SSD1306_INITCMDS[cmd_idx])) return cmd_idx;
	            cmd_idx++;
	        }
	    }
	    return 0;
}

uint8_t SSD1306_Clear(SSD1306_HandleTypeDef *hssd) {
	hssd->str_cursor = 0;						// Reset the cursor to top-left
	memset(hssd->vram, 0x00, CACHE_SIZE_MEM);	// clear vram
	return 0;
}


uint8_t SSD1306_Update(SSD1306_HandleTypeDef *hssd) {
	hssd->vram_full[0] = DATA_STREAM; 			// Identify the outgoing data as a stream
	return HAL_I2C_Master_Transmit_DMA(hssd->i2c_handle, (hssd->address) << 1, hssd->vram_full, CACHE_SIZE_MEM + 1);
}

uint8_t SSD1306_DrawChar(SSD1306_HandleTypeDef *hssd, char ch) {
	// Account for newline
	if (ch == '\n') {
		hssd->str_cursor = (hssd->str_cursor/128)*128;
		return SUCCESS;
	}

	char newC = ch - 32;					// offset the index-space so the ASCII code aligns with the font table
	if (ch >= 97 && ch <= 122) newC -= 32;	// convert lowercase to uppercase
	if (ch >= 123)             newC -= 26;	// remap the rest to align with the font table

	if (newC > 126) return ERROR;	// char is unable to be rendered

	uint8_t len = ALPHNUM[newC*8];
	for (uint8_t i = 0; i < len; i++) {

		uint8_t drawbyte = ALPHNUM[(newC*8)+1 + i];
		if (hssd->draw_inverted) drawbyte ^= 0xFF;

		if (!hssd->draw_scale) hssd->vram[hssd->str_cursor + i] = drawbyte;
		else {
			uint8_t drawbyte_1 =  (drawbyte & 0b00000001)       | ((drawbyte & 0b00000001) << 1) |
								 ((drawbyte & 0b00000010) << 1) | ((drawbyte & 0b00000010) << 2) |
								 ((drawbyte & 0b00000100) << 2) | ((drawbyte & 0b00000100) << 3) |
								 ((drawbyte & 0b00001000) << 3) | ((drawbyte & 0b00001000) << 4);

			uint8_t drawbyte_2 = ((drawbyte & 0b00010000) >> 4) | ((drawbyte & 0b00010000) >> 3) |
								 ((drawbyte & 0b00100000) >> 3) | ((drawbyte & 0b00100000) >> 2) |
								 ((drawbyte & 0b01000000) >> 2) | ((drawbyte & 0b01000000) >> 1) |
								 ((drawbyte & 0b10000000) >> 1) | (drawbyte & 0b10000000);

			hssd->vram[hssd->str_cursor + i*2      ] = drawbyte_1;
			hssd->vram[hssd->str_cursor + i*2 + 1  ] = drawbyte_1;
			hssd->vram[hssd->str_cursor + i*2 + 128] = drawbyte_2;
			hssd->vram[hssd->str_cursor + i*2 + 129] = drawbyte_2;
		}
	}
	//memcpy(hssd->vram + hssd->str_cursor, ALPHNUM+(newC*8)+1, len);
	hssd->str_cursor += len;
	if (hssd->draw_scale)
		hssd->str_cursor += len;
	return SUCCESS;
}


uint8_t SSD1306_DrawString(SSD1306_HandleTypeDef *hssd, char *str, uint8_t length) {
	uint8_t start_line = hssd->str_cursor / 120;
	for (uint8_t i = 0; i < length; i++) {
		if (hssd->str_cursor / 128 > start_line) break;
	    if (SSD1306_DrawChar(hssd, str[i])) hssd->str_cursor += 0x00;
	}

	return SUCCESS;
}
