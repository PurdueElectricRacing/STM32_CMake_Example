/**
 * @file menu_system.h
 * @brief Implementation of menu system for LCD display interface
 * 
 * Provides functions for managing menu navigation, element styling,
 * and user interaction with the LCD display menu system.
 * 
 * @author Irving Wang (wang5952@purdue.edu)
 */

#ifndef MENU_SYSTEM_H
#define MENU_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>

// Element types
typedef enum {
    ELEMENT_VAL = 0,    // Numeric value
    ELEMENT_FLT,    // Float value
    ELEMENT_BAR,    // (not supported but easy to implement)
    ELEMENT_BUTTON, // Button type
    ELEMENT_OPTION, // On/off toggle
    ELEMENT_LIST    // Item in a list
} element_type_t;

// Element states
typedef enum {
    STATE_NORMAL = 0,
    STATE_HOVER,
    STATE_SELECTED
} element_state_t;

// Element structure
typedef struct {
    element_type_t type;
    element_state_t state;
    char* object_name;          // Nextion object name
    void (*on_change)(void);    // Callback when value changes
    uint8_t current_value;      // Current value for numeric types or on/off state for toggles
    uint8_t min_value;          // Minimum value for numeric types
    uint8_t max_value;          // Maximum value for numeric types
    uint8_t increment;          // Increment for numeric types
} menu_element_t; // todo eventually make this const (to store in flash)

// Page structure
typedef struct {
    menu_element_t* elements;   // Array of elements
    uint8_t num_elements;       // Number of elements in array
    uint8_t current_index;      // Currently selected element index
    bool is_element_selected;   // Is an element currently selected?
    bool saved;                 // Generic saved state flag
} menu_page_t;

// Navigation functions
void MS_moveUp(menu_page_t* page);
void MS_moveDown(menu_page_t* page);
void MS_select(menu_page_t* page);

// Value modification functions
void MS_incrementValue(menu_element_t* element);
void MS_decrementValue(menu_element_t* element);

void MS_refreshPage(menu_page_t *page);
int MS_listGetSelected(menu_page_t *page);

#endif // MENU_SYSTEM_H