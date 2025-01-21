#ifndef __NEXTION_H__
#define __NEXTION_H__

#include <string.h>
#include <stdio.h>
#include "common/queue/queue.h"

// Size Definitions
#define NXT_STR_SIZE        100 // ! Important: Used for usart queue size, issues arise if less than len of longest fault msg
#define NXT_CMD_TERM "\xFF\xFF\xFF" // Serial insturcitons must be terminated with 3 bytes 0xFF
#define ASCII_OFFSET 		48

// Color Definitions in 565 format
#define RED    63488
#define YELLOW 65504
#define GREEN  4065
#define RACE_GREEN 1376
#define WHITE 65535
#define INFO_GRAY 48631
#define BLACK 0
#define ORANGE 64512

// Nextion Command Strings
#define NXT_BACKGROUND_COLOR ".bco="
#define NXT_FONT_COLOR       ".pco="
#define NXT_VALUE            ".val="
#define NXT_TEXT             ".txt="
#define NXT_BORDERW          ".borderw=" // Cannot modify the borderw of the TEXT object without intelligent series
#define NXT_PAGE             "page "
#define NXT_BRIGHTNESS       "dims="
#define NXT_BAUD             "bauds="

// Function Prototypes
void set_float(char* obj_name, char* param, float num, uint8_t precision);
void set_value(char* obj_name, uint16_t val);
void set_background(char* obj_name, uint16_t val);
void set_font_color(char* obj_name, uint16_t val);
void set_border_width(char* obj_name, uint16_t val);
void set_text(char* obj_name, char* text);
void set_textf(char* obj_name, const char* format, ...);
void set_page(char* page_name);
void set_brightness(uint8_t brightness);
void set_baud(uint32_t baud);

#endif