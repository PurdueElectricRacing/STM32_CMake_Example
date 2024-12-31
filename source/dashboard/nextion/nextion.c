/**
 * @file nextion.c
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

#include "nextion.h"

#include <stdarg.h>

extern q_handle_t q_tx_usart;

/**
 * @brief Sets a flag on a Nextion display object.
 *
 * @param obj_name The name of the Nextion display object.
 * @param enable A boolean value indicating whether to enable (1) or disable (0) the flag.
 */
void set_flag(char* obj_name, uint8_t enable)
{
  if (enable) set_value(obj_name, FLAG_ENABLED_PIC);
  else set_value(obj_name, FLAG_DISABLED_PIC);
}

/**
 * @brief Sets a float value to a specified object parameter on the Nextion display.
 *
 * @param obj_name The name of the object on the Nextion display.
 * @param param The parameter of the object to set the float value to.
 * @param num The float value to set.
 * @param precision The number of decimal places to include in the formatted float value.
 */
void set_float(char* obj_name, char* param, float num, uint8_t precision)
{
  char result[NXT_STR_SIZE];
  snprintf(result, sizeof(result), "%s%s\"%.*f\"%s", obj_name, param, precision, num, NXT_CMD_TERM);
  qSendToBack(&q_tx_usart, (uint16_t *) result);
}

/**
 * @brief Sets a value to a specified object on the Nextion display.
 *
 * @param obj_name The name of the object on the Nextion display.
 * @param val The value to set for the specified object.
 */
void set_value(char* obj_name, uint16_t val) 
{
  char result[NXT_STR_SIZE];
  snprintf(result, sizeof(result), "%s%s%d%s", obj_name, NXT_VALUE, val, NXT_CMD_TERM);
  qSendToBack(&q_tx_usart, (uint16_t *) result);
}

/**
 * @brief Sets the background color of a specified object on the Nextion display.
 *
 * @param obj_name The name of the object on the Nextion display.
 * @param val The background color value to set for the specified object.
 */
void set_background(char* obj_name, uint16_t val) 
{
  char result[NXT_STR_SIZE];
  snprintf(result, sizeof(result), "%s%s%d%s", obj_name, NXT_BACKGROUND_COLOR, val, NXT_CMD_TERM);
  qSendToBack(&q_tx_usart, (uint16_t *) result);
}

/**
 * @brief Sets the font color of a specified object on the Nextion display.
 *
 * @param obj_name The name of the object on the Nextion display.
 * @param val The font color value to set for the specified object.
 */
void set_font_color(char* obj_name, uint16_t val) 
{
  char result[NXT_STR_SIZE];
  snprintf(result, sizeof(result), "%s%s%d%s", obj_name, NXT_FONT_COLOR, val, NXT_CMD_TERM);
  qSendToBack(&q_tx_usart, (uint16_t *) result);
}

/**
 * @brief Sets the text of a Nextion display object.
 *
 * @param obj_name The name of the Nextion object (e.g., "t0").
 * @param text The text to set for the specified obect.
 */
void set_text(char* obj_name, char* text)
{
  char result[NXT_STR_SIZE];
  snprintf(result, sizeof(result), "%s%s\"%s\"%s", obj_name, NXT_TEXT, text, NXT_CMD_TERM);
  qSendToBack(&q_tx_usart, (uint16_t *) result);
}

/**
 * @brief Sets the text of a Nextion display object using printf-style formatting.
 *
 * @param obj_name The name of the Nextion object (e.g., "t0").
 * @param format Printf-style format string.
 * @param ... Variable arguments for format string.
 */
void set_textf(char* obj_name, const char* format, ...)
{
  char formatted[NXT_STR_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(formatted, sizeof(formatted), format, args);
  va_end(args);
  
  set_text(obj_name, formatted);
}

/**
 * @brief Sets the current page on the Nextion display.
 *
 * @param page_name The name of the page to set on the Nextion display.
 */
void set_page(char* page_name) {
  char result[NXT_STR_SIZE];
  snprintf(result, sizeof(result), "%s%s%s", NXT_PAGE, page_name, NXT_CMD_TERM);
  qSendToBack(&q_tx_usart, (uint16_t *) result);
}

/**
 * @brief Sets the brightness of the Nextion display.
 *
 * @param percentage The desired brightness level as a percentage (0-100).
 */
void set_brightness(uint8_t percentage) {
  if (percentage > 100) percentage = 100;
  char result[NXT_STR_SIZE];
  snprintf(result, sizeof(result), "%s%d%s", NXT_BRIGHTNESS, percentage, NXT_CMD_TERM);
  qSendToBack(&q_tx_usart, (uint16_t *) result);
}

/**
 * @brief Sets the baud rate for the Nextion display.
 *
 * @param baud The desired baud rate to set for the Nextion display.
 */
void set_baud(uint32_t baud) {
  char result[NXT_STR_SIZE];
  snprintf(result, sizeof(result), "%s%d%s", NXT_BAUD, baud, NXT_CMD_TERM);
  qSendToBack(&q_tx_usart, (uint16_t *) result);
}

