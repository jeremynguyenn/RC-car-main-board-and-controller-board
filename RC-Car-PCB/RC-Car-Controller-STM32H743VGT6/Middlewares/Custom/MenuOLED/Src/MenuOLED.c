/*
 * MenuOLED.c
 *
 *  Created on: May 25, 2025
 *      Author: Loochis
 */

#include "MenuOLED.h"

// FUNCS
// ------------------------------------------------------------------------------------

uint8_t* AllocateValueArr(uint8_t num) {
	return (uint8_t*)malloc(sizeof(uint8_t)*num);
}

uint8_t* AllocateString(uint8_t* src) {
	uint8_t str_tmp[100];
	uint8_t* dst = (uint8_t*)malloc(strlen(src)+1);
	strcpy(dst, src);
	return dst;
}

uint8_t** AllocateStringArr(uint8_t num) {
	return (uint8_t**)malloc(sizeof(uint8_t*)*num);
}

Menu_Property* AllocateProperties(uint8_t num) {
	return (Menu_Property*)malloc(sizeof(Menu_Property)*num);
}

Menu_Page* AllocatePages(uint8_t num) {
	return (Menu_Page*)malloc(sizeof(Menu_Page)*num);
}

uint8_t MENU_Init(Menu_HandleTypeDef *hmenu) {
	// Initialize the state packet
	hmenu->state_packet = AllocateValueArr(64);
	memset(hmenu->state_packet, 0x00, 64);

	// Allocate pages
	hmenu->num_pages = 3;
	hmenu->pages = AllocatePages(hmenu->num_pages);

	// PAGE 0 (CAMERA)
	hmenu->pages[0].title = AllocateString("CAMERA");

	hmenu->pages[0].num_properties = 3;
	hmenu->pages[0].properties = AllocateProperties(hmenu->pages[0].num_properties);

	// Camera Quality
	hmenu->pages[0].properties[0].name = AllocateString("QUALITY");
	hmenu->pages[0].properties[0].packet_byte = OP_CAMERA_QUALITY;

	hmenu->pages[0].properties[0].num_options = 4;
	hmenu->pages[0].properties[0].option_names = AllocateStringArr(hmenu->pages[0].properties[0].num_options);
	hmenu->pages[0].properties[0].option_names[0] = AllocateString("LOW");
	hmenu->pages[0].properties[0].option_names[1] = AllocateString("MED");
	hmenu->pages[0].properties[0].option_names[2] = AllocateString("HIGH");
	hmenu->pages[0].properties[0].option_names[3] = AllocateString("BEST");

	// Camera Vertical Shift
	hmenu->pages[0].properties[1].name = AllocateString("SHOW MS");
	hmenu->pages[0].properties[1].packet_byte = OP_CAMERA_FRAMETIME;

	hmenu->pages[0].properties[1].num_options = 2;
	hmenu->pages[0].properties[1].option_names = AllocateStringArr(hmenu->pages[0].properties[1].num_options);
	hmenu->pages[0].properties[1].option_names[0] = AllocateString("[\x83\x83]");
	hmenu->pages[0].properties[1].option_names[1] = AllocateString("[\x80\x80]");

	// Camera Mode
	hmenu->pages[0].properties[2].name = AllocateString("ENCODING");
	hmenu->pages[0].properties[2].packet_byte = OP_CAMERA_ENCODING;

	hmenu->pages[0].properties[2].num_options = 2;
	hmenu->pages[0].properties[2].option_names = AllocateStringArr(hmenu->pages[0].properties[2].num_options);
	hmenu->pages[0].properties[2].option_names[0] = AllocateString("JPEG");
	hmenu->pages[0].properties[2].option_names[1] = AllocateString("RAW");

	// PAGE 1 (LIGHTS)
	hmenu->pages[1].title = AllocateString("LIGHTING");

	hmenu->pages[1].num_properties = 4;
	hmenu->pages[1].properties = AllocateProperties(hmenu->pages[1].num_properties);

	// Pre-allocate the percent strings
	uint8_t *percentStr[5];
	percentStr[0] = AllocateString("[\x80\x80\x80\x80]");
	percentStr[1] = AllocateString("[\x83\x80\x80\x80]");
	percentStr[2] = AllocateString("[\x83\x83\x80\x80]");
	percentStr[3] = AllocateString("[\x83\x83\x83\x80]");
	percentStr[4] = AllocateString("[\x83\x83\x83\x83]");

	hmenu->pages[1].properties[0].name = AllocateString("HEADLIGHTS");
	hmenu->pages[1].properties[0].packet_byte = 3;

	hmenu->pages[1].properties[0].num_options = 5;
	hmenu->pages[1].properties[0].option_names = AllocateStringArr(hmenu->pages[1].properties[0].num_options);

	for (uint8_t i = 0; i < 5; i++)
		hmenu->pages[1].properties[0].option_names[i] = percentStr[i];

	hmenu->pages[1].properties[1].name = AllocateString("INT. R");
	hmenu->pages[1].properties[1].packet_byte = 4;

	hmenu->pages[1].properties[1].num_options = 5;
	hmenu->pages[1].properties[1].option_names = AllocateStringArr(hmenu->pages[1].properties[1].num_options);

	for (uint8_t i = 0; i < 5; i++)
		hmenu->pages[1].properties[1].option_names[i] = percentStr[i];

	hmenu->pages[1].properties[2].name = AllocateString("INT. G");
	hmenu->pages[1].properties[2].packet_byte = 5;

	hmenu->pages[1].properties[2].num_options = 5;
	hmenu->pages[1].properties[2].option_names = AllocateStringArr(hmenu->pages[1].properties[2].num_options);

	for (uint8_t i = 0; i < 5; i++)
		hmenu->pages[1].properties[2].option_names[i] = percentStr[i];

	hmenu->pages[1].properties[3].name = AllocateString("INT. B");
	hmenu->pages[1].properties[3].packet_byte = 6;

	hmenu->pages[1].properties[3].num_options = 5;
	hmenu->pages[1].properties[3].option_names = AllocateStringArr(hmenu->pages[1].properties[3].num_options);

	for (uint8_t i = 0; i < 5; i++)
		hmenu->pages[1].properties[3].option_names[i] = percentStr[i];

	return 0;
}

uint8_t MENU_Draw(Menu_HandleTypeDef *hmenu, uint32_t delta_t) {
	// Do the animations
	hmenu->page_anim++;
	if (hmenu->page_anim == 0xFF)
		hmenu->page_anim = 0xF1; // Loop the last 16 animation frames


	if (hmenu->property_anim != 0xFF)
		hmenu->property_anim++;

	if (hmenu->current_page == 0) {
		// STATUS PAGE, SPECIAL RULES
		uint8_t tmp_msg[20];

		// Draw the title CAR on the left screen
		hmenu->ssdL_handle->str_cursor = (128 - 7*6);
		MENU_AnimateString(hmenu, hmenu->ssdL_handle,
				"VEHICLE",
				hmenu->page_anim, 0);

		// Draw the left screen stats

		if (hmenu->alert_voltage_car && (hmenu->page_anim % 4) >= 2)
					hmenu->ssdL_handle->draw_inverted = 1;

		hmenu->ssdL_handle->str_cursor = 16 + 2*128;
		sprintf(tmp_msg, "% 5.2f V  ", hmenu->voltage_car);
		MENU_AnimateString(hmenu, hmenu->ssdL_handle,
						tmp_msg,
						hmenu->page_anim, 2);

		hmenu->ssdL_handle->draw_inverted = 0;
		if (hmenu->alert_current_car && (hmenu->page_anim % 4) >= 2)
			hmenu->ssdL_handle->draw_inverted = 1;

		hmenu->ssdL_handle->str_cursor = 16 + 3*128;
		sprintf(tmp_msg, "% 5.2f A ", hmenu->current_car);
		MENU_AnimateString(hmenu, hmenu->ssdL_handle,
						tmp_msg,
						hmenu->page_anim, 3);

		hmenu->ssdL_handle->draw_inverted = 0;

		hmenu->ssdL_handle->str_cursor = 60 + 2*128;
		hmenu->ssdL_handle->draw_scale = 1;
		sprintf(tmp_msg, "% 4.0f", hmenu->voltage_car*hmenu->current_car);
		MENU_AnimateString(hmenu, hmenu->ssdL_handle,
						tmp_msg,
						hmenu->page_anim, 6);

		hmenu->ssdL_handle->str_cursor = 114 + 2*128;
		MENU_AnimateString(hmenu, hmenu->ssdL_handle,
						"W",
						hmenu->page_anim, 10);

		hmenu->ssdL_handle->draw_scale = 0;

		// BATTERY

		hmenu->ssdL_handle->str_cursor = 16 + 5*128;
		sprintf(tmp_msg, "BAT:  % 3.0f ", hmenu->bat_perc_car);
		MENU_AnimateString(hmenu, hmenu->ssdL_handle,
						tmp_msg,
						hmenu->page_anim, 6);


		if (hmenu->bat_time_car >= 0) {
			hmenu->ssdL_handle->str_cursor = 16 + 6*128;
			sprintf(tmp_msg, "TIME: % 3.0f ", hmenu->bat_time_car);
			MENU_AnimateString(hmenu, hmenu->ssdL_handle,
							tmp_msg,
							hmenu->page_anim, 8);

			hmenu->ssdL_handle->str_cursor = 16 + 7*128;
			sprintf(tmp_msg, "CHARGE");
			MENU_AnimateString(hmenu, hmenu->ssdL_handle,
						tmp_msg,
						hmenu->page_anim, 10);
		}
		else {
			hmenu->ssdL_handle->str_cursor = 16 + 6*128;
			sprintf(tmp_msg, "TIME: % 3.0f ", -hmenu->bat_time_car);
			MENU_AnimateString(hmenu, hmenu->ssdL_handle,
							tmp_msg,
							hmenu->page_anim, 8);

			hmenu->ssdL_handle->str_cursor = 16 + 7*128;
			sprintf(tmp_msg, "DISCHARGE");
			MENU_AnimateString(hmenu, hmenu->ssdL_handle,
						tmp_msg,
						hmenu->page_anim, 10);
		}

		// Draw the title CON on the right screen
		hmenu->ssdR_handle->str_cursor = 0;
		MENU_AnimateString(hmenu, hmenu->ssdR_handle,
				"CONTROLLER",
				hmenu->page_anim, 0);

		// Draw the right screen stats

		if (hmenu->alert_voltage_con && (hmenu->page_anim % ALERT_BLINKFREQ) >= ALERT_BLINKDUTY)
			hmenu->ssdR_handle->draw_inverted = 1;

		hmenu->ssdR_handle->str_cursor = 2*128;
		sprintf(tmp_msg, "% 5.2f V ", hmenu->voltage_con);
		MENU_AnimateString(hmenu, hmenu->ssdR_handle,
						tmp_msg,
						hmenu->page_anim, 2);

		hmenu->ssdR_handle->draw_inverted = 0;
		if (hmenu->alert_current_con && (hmenu->page_anim % ALERT_BLINKFREQ) >= ALERT_BLINKDUTY)
			hmenu->ssdR_handle->draw_inverted = 1;

		hmenu->ssdR_handle->str_cursor = 3*128;
		sprintf(tmp_msg, "% 5.2f A ", hmenu->current_con);
		MENU_AnimateString(hmenu, hmenu->ssdR_handle,
						tmp_msg,
						hmenu->page_anim, 3);

		hmenu->ssdR_handle->draw_inverted = 0;

		hmenu->ssdR_handle->str_cursor = 44 + 2*128;
		hmenu->ssdR_handle->draw_scale = 1;
		sprintf(tmp_msg, "% 4.0f", hmenu->voltage_con*hmenu->current_con);
		MENU_AnimateString(hmenu, hmenu->ssdR_handle,
						tmp_msg,
						hmenu->page_anim, 6);

		hmenu->ssdR_handle->str_cursor = 98 + 2*128;
		MENU_AnimateString(hmenu, hmenu->ssdR_handle,
						"W",
						hmenu->page_anim, 10);
		hmenu->ssdR_handle->draw_scale = 0;

		// BATTERY

		// Draw the divider
		hmenu->ssdR_handle->draw_scale = 1;

		hmenu->ssdR_handle->str_cursor = 4*128 + 2;
		sprintf(tmp_msg, "---------");
		MENU_AnimateString(hmenu, hmenu->ssdR_handle,
						tmp_msg,
						hmenu->page_anim, 6);

		hmenu->ssdR_handle->str_cursor = 5*128 + 80;
		sprintf(tmp_msg, "---");
		MENU_AnimateString(hmenu, hmenu->ssdR_handle,
						tmp_msg,
						hmenu->page_anim, 6);

		hmenu->ssdR_handle->draw_scale = 0;

		// Draw the bats label
		hmenu->ssdR_handle->draw_inverted = 1;
		hmenu->ssdR_handle->str_cursor = 5*128 + 74;
		sprintf(tmp_msg, "\x84 BATS ");
		MENU_AnimateString(hmenu, hmenu->ssdR_handle,
						tmp_msg,
						hmenu->page_anim, 6);
		hmenu->ssdR_handle->draw_inverted = 0;

		if (hmenu->bat_time_con == 0.0) {
			// CHARGING
			hmenu->ssdR_handle->draw_scale = 1;
			hmenu->ssdR_handle->str_cursor = 6*128 + 10;
			sprintf(tmp_msg, "CHRG");
			MENU_AnimateString(hmenu, hmenu->ssdR_handle,
							tmp_msg,
							hmenu->page_anim, 6);
			hmenu->ssdR_handle->draw_scale = 0;

		} else {
			// DISCHARGING
			hmenu->ssdR_handle->str_cursor = 6*128;
			if ((hmenu->alert_battery_con) && (hmenu->page_anim % ALERT_BLINKFREQ) >= ALERT_BLINKDUTY)
				hmenu->ssdR_handle->draw_inverted = 1;

			sprintf(tmp_msg, "% 4.1f %%", hmenu->bat_perc_con);
			MENU_AnimateString(hmenu, hmenu->ssdR_handle,
							tmp_msg,
							hmenu->page_anim, 7);

			hmenu->ssdR_handle->draw_inverted = 0;

			hmenu->ssdR_handle->str_cursor = 7*128;
			sprintf(tmp_msg, "% 4.1f M", hmenu->bat_time_con);
			MENU_AnimateString(hmenu, hmenu->ssdR_handle,
							tmp_msg,
							hmenu->page_anim, 8);
		}

		hmenu->ssdR_handle->str_cursor = 7*128 + 68;
		sprintf(tmp_msg, "% 3.1f *C", hmenu->bat_temp_con);
		MENU_AnimateString(hmenu, hmenu->ssdR_handle,
						tmp_msg,
						hmenu->page_anim, 8);


	} else {
		// REGULAR PAGE, ADJUST INDEX

		Menu_Page 	  activePage = hmenu->pages[hmenu->current_page-1];
		Menu_Property activeProperty = activePage.properties[hmenu->current_property];

		// Draw the title on the left
		hmenu->ssdL_handle->str_cursor = (128 - strlen(activePage.title)*6);
		MENU_AnimateString(hmenu, hmenu->ssdL_handle,
				activePage.title,
				hmenu->page_anim, 0);


		for (uint8_t i = 0; i < activePage.num_properties; i++) {
			// Draw the properties
			// Compute offset using property anim
			hmenu->ssdL_handle->str_cursor = 16 + (2+i)*128;
			if (i == hmenu->current_property) {
				uint8_t num_bars = hmenu->property_anim;
				if (num_bars > 3) num_bars = 3;
				SSD1306_DrawString(hmenu->ssdL_handle, ">> ", num_bars);
			}

			// De-animate the previous property
			if (i == hmenu->last_property && hmenu->property_anim/2 <= 2) {
				uint8_t num_bars = 2 - hmenu->property_anim/2;
				if (num_bars > 2) num_bars = 2;
				SSD1306_DrawString(hmenu->ssdL_handle, ">> ", num_bars);
			}

			MENU_AnimateString(hmenu, hmenu->ssdL_handle,
					activePage.properties[i].name,
					hmenu->page_anim, 6+i*2);

			// Draw the values these properties have
			uint8_t op_value = hmenu->state_packet[activePage.properties[i].packet_byte];
			if (op_value < activePage.properties[i].num_options) {
				hmenu->ssdL_handle->str_cursor = (3+i)*128 - strlen(activePage.properties[i].option_names[op_value])*6;
				MENU_AnimateString(hmenu, hmenu->ssdL_handle,
						activePage.properties[i].option_names[op_value],
						hmenu->page_anim, 6+i*2);
			}
		}

		// Draw the selected property on the RIGHT
		hmenu->ssdR_handle->str_cursor = 0;
		MENU_AnimateString(hmenu, hmenu->ssdR_handle,
					activeProperty.name,
					hmenu->property_anim, 1);

		// Draw the selected option on the right
		uint8_t op_value = hmenu->state_packet[activeProperty.packet_byte];
		if (op_value < activeProperty.num_options) {
			hmenu->ssdR_handle->str_cursor = 2*128;
			SSD1306_DrawString(hmenu->ssdR_handle, "> ", 2);
			SSD1306_DrawString(hmenu->ssdR_handle, activeProperty.option_names[op_value], strlen(activeProperty.option_names[op_value]));
			SSD1306_DrawString(hmenu->ssdR_handle, " <", 2);
		}
	}
}

void MENU_ParseInput(Menu_HandleTypeDef *hmenu, uint8_t inputs[4]) {
	if (inputs[0]) {
		if (hmenu->current_page == 0) {
			hmenu->current_page = hmenu->num_pages - 1;
		} else {
			hmenu->current_page--;
		}
		hmenu->page_anim = 0;
		hmenu->property_anim = 0;
		hmenu->current_property = 0;
		hmenu->last_property = 0xFF;
	} else if (inputs[1]) {
		if (hmenu->current_page == hmenu->num_pages - 1) {
			hmenu->current_page = 0;
		} else {
			hmenu->current_page++;
		}
		hmenu->page_anim = 0;
		hmenu->property_anim = 0;
		hmenu->current_property = 0;
		hmenu->last_property = 0xFF;
	}

	if (inputs[3]) {
		hmenu->last_property = hmenu->current_property;
		if (hmenu->current_property == hmenu->pages[hmenu->current_page].num_properties - 1)
			hmenu->current_property = 0;
		else
			hmenu->current_property++;
		hmenu->property_anim = 0;
	}

	if (inputs[2] && hmenu->current_page > 0) {
		Menu_Property activeProperty = hmenu->pages[hmenu->current_page-1].properties[hmenu->current_property];
		uint8_t propertyByte = activeProperty.packet_byte;
		if (hmenu->state_packet[propertyByte] == activeProperty.num_options - 1)
			hmenu->state_packet[propertyByte] = 0;
		else
			hmenu->state_packet[propertyByte]++;
	}
}

void MENU_AnimateString(Menu_HandleTypeDef *hmenu, SSD1306_HandleTypeDef *hssd, uint8_t *str, uint8_t anim_val, uint8_t anim_start) {
	if (hmenu->page_anim < anim_start) return;
	uint8_t min_len = strlen(str);
	if (min_len > anim_val - anim_start)
		min_len = anim_val - anim_start;

	SSD1306_DrawString(hssd, str, min_len);
}

// UTIL
float Lerp(float a, float b, float t) {
	if (t >= 1) return b;
	if (t <= 0) return a;
	return a*(1.0-t) + b*t;
}
