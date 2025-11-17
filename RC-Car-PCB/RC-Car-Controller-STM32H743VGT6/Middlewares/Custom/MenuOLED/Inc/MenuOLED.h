/*
 * MenuOLED.h
 *
 *  Created on: May 25, 2025
 *      Author: Loochis
 */

#ifndef CUSTOM_MENUOLED_INC_MENUOLED_H_
#define CUSTOM_MENUOLED_INC_MENUOLED_H_

#include "SSD1306.h"

// DEFINITIONS
// ------------------------------------------------------------------------------------

#define ALERT_BLINKFREQ 8
#define ALERT_BLINKDUTY 4

#define OP_CAMERA_ENCODING  0x00
#define OP_CAMERA_QUALITY   0x01
#define OP_CAMERA_FRAMETIME 0x02

#define RESERVE_LTRACK_MAG	0x07
#define RESERVE_RTRACK_MAG	0x08
#define RESERVE_LTRACK_DIR	0x09
#define RESERVE_RTRACK_DIR	0x0A
// STRUCTS
// ------------------------------------------------------------------------------------

typedef struct
{
	uint8_t					*name;			// string: name of this property

	uint8_t					packet_byte;	// which byte of the packet stores this value

	uint8_t					num_options;	// number of values this property can choose from

	uint8_t					**option_names;	// array of strings: values this property can choose from

} Menu_Property;

typedef struct
{

	uint8_t					*title;			// string: title of this menu page

	uint8_t					num_properties;	// number of properties this menu page has

	Menu_Property			*properties;	// array of properties (contiguous)

} Menu_Page;

typedef struct
{
	SSD1306_HandleTypeDef 	*ssdL_handle;	// ptr to SSD1306_HandleTypeDef which renders the LEFT menu

	SSD1306_HandleTypeDef 	*ssdR_handle;	// ptr to SSD1306_HandleTypeDef which renders the RIGHT menu

	uint8_t					num_pages;		// number of pages this menu has

	Menu_Page				*pages;			// array of pages (contiguous)

	// Menu state variables

	uint8_t					*state_packet;	// Packet which stores the whole state of the menu

	uint8_t					current_page;

	uint8_t 				page_anim;

	uint8_t					current_property;

	uint8_t					last_property;

	uint8_t 				property_anim;

	// Stats Values

	float					current_con;

	float					current_car;

	float					voltage_con;

	float					voltage_car;

	float					bat_perc_con;

	float					bat_perc_car;

	float					bat_time_con;

	float					bat_time_car;

	float					bat_temp_con;

	float					bat_temp_car;

	uint8_t 				alert_current_con;

	uint8_t 				alert_current_car;

	uint8_t 				alert_voltage_con;

	uint8_t 				alert_voltage_car;

	uint8_t 				alert_battery_con;

	uint8_t 				alert_battery_car;

} Menu_HandleTypeDef;

// FUNCS
// ------------------------------------------------------------------------------------

// Utilities
uint8_t*	   	AllocateValueArr(uint8_t num);
uint8_t*       	AllocateString(uint8_t* str);
uint8_t**      	AllocateStringArr(uint8_t num);
Menu_Property* 	AllocateProperties(uint8_t num);
Menu_Page*     	AllocatePages(uint8_t num);

uint8_t MENU_Init(Menu_HandleTypeDef *hmenu);
uint8_t MENU_Draw(Menu_HandleTypeDef *hmenu, uint32_t delta_t);

void MENU_ParseInput(Menu_HandleTypeDef *hmenu, uint8_t inputs[4]);
void MENU_AnimateString(Menu_HandleTypeDef *hmenu, SSD1306_HandleTypeDef *hssd, uint8_t *str, uint8_t anim_val, uint8_t anim_start);

float Lerp(float a, float b, float t);

#endif /* CUSTOM_MENUOLED_INC_MENUOLED_H_ */
