/**
 * @file nextion.h
 * @brief Nextion display driver interface
 * 
 * Interface for controlling Nextion display modules through serial communication.
 * 
 * @author Matthew Flanagan (matthewdavidflanagan@outlook.com)
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @author Irving Wang (wang5952@purdue.edu)
 * 
 * Original implementation by Matthew Flanagan
 * Converted for current use by Luke Oxley
 * Expanded by Irving Wang
 */

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
void NXT_setFloat(char* obj_name, char* param, float num, uint8_t precision);
void NXT_setValue(char* obj_name, uint16_t val);
void NXT_setBackground(char* obj_name, uint16_t val);
void NXT_setFontColor(char* obj_name, uint16_t val);
void NXT_setBorderWidth(char* obj_name, uint16_t val);
void NXT_setText(char* obj_name, char* text);
void NXT_setTextFormatted(char* obj_name, const char* format, ...);
void NXT_setPage(char* page_name);

// Configuration Functions
void NXT_setBrightness(uint8_t brightness);
void NXT_setBaud(uint32_t baud);

#endif