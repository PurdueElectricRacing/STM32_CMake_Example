/**
 * @file menu_system.c
 * @brief Implementation of menu system for LCD display interface
 * 
 * Provides functions for managing menu navigation, element styling,
 * and user interaction with the LCD display menu system.
 * 
 * @author Irving Wang (wang5952@purdue.edu)
 */

#include "menu_system.h"

#include "nextion.h"
#include <stdint.h>

// Purdue color palette in 565 format
#define RUSH 56640          // bright gold
#define BOILERMAKER 52690   // pale gold
#define FIELD 56776         // light gold
#define STEEL 21196         // dark grey
#define COOL_GRAY 27535     // medium grey
#define STEAM 50680         // light grey

/**
 * @brief Applies normal (default) styling to a menu element
 * @param element Pointer to the menu element to be styled
 */
void MS_setStyleNormal(menu_element_t *element) {
    NXT_setBackground(element->object_name, STEEL);
    NXT_setFontColor(element->object_name, WHITE);
    // change to set border to indicate selected? (requires intelligent series)
    // set_border_width(element->object_name, 0);
}

/**
 * @brief Applies hover styling effect to a menu element
 * @param element Pointer to the menu element to be styled
 */
void MS_setStyleHover(menu_element_t *element) {
    NXT_setBackground(element->object_name, STEAM);
    NXT_setFontColor(element->object_name, BLACK);
    // change to set border to indicate selected? (requires intelligent series)
    // set_border_width(element->object_name, 3);
}

/**
 * @brief Applies selected state styling to a menu element
 * @param element Pointer to the menu element to be styled
 */
void MS_setStyleSelected(menu_element_t *element) {
    NXT_setBackground(element->object_name, RUSH);
    NXT_setFontColor(element->object_name, WHITE);
}

/**
 * @brief Moves menu selection up or increments selected element value
 *
 * @param page Pointer to the menu page structure
 */
void MS_moveUp(menu_page_t *page) {
    if (page->is_element_selected) {
        MS_incrementValue(&page->elements[page->current_index]);
        return;
    }

    // Clear current element styling
    MS_setStyleNormal(&page->elements[page->current_index]);

    // Move to previous element
    page->current_index = (page->current_index - 1 + page->num_elements) % page->num_elements;

    // Style new element as hovered
    MS_setStyleHover(&page->elements[page->current_index]);
}

/**
 * @brief Moves menu selection down or increments selected element value
 *
 * @param page Pointer to the menu page structure
 */
void MS_moveDown(menu_page_t *page) {
    if (page->is_element_selected) {
        MS_decrementValue(&page->elements[page->current_index]);
        return;
    }

    // Clear current element styling
    MS_setStyleNormal(&page->elements[page->current_index]);

    // Move to next element
    page->current_index = (page->current_index + 1) % page->num_elements;

    // Style new element as hovered
    MS_setStyleNormal(&page->elements[page->current_index]);
}

/**
 * @brief Handles menu element selection and actions
 *
 * @param page Pointer to the menu page
 */
void MS_select(menu_page_t *page) {
    menu_element_t* current = &page->elements[page->current_index];

    if (page->is_element_selected) {
        // Deselect element
        page->is_element_selected = false;
        MS_setStyleHover(current);
        
        // Call onChange if defined
        if (current->on_change != NULL) {
            current->on_change();
        }
        return;
    }

    // Select element if it's a selectable type
    switch (current->type) {
        case ELEMENT_BUTTON:
            if (current->on_change != NULL) {
                current->on_change();
            }
            break;
        case ELEMENT_LIST:
            // Clear other options
            for (uint8_t i = 0; i < page->num_elements; i++) {
                if (page->elements[i].type == ELEMENT_LIST) {
                    page->elements[i].current_value = 0;
                    MS_setStyleNormal(&page->elements[i]);
                }
            }
            current->current_value = 1;
            if (current->on_change != NULL) {
                current->on_change();
            }
            MS_setStyleSelected(current);
            break;
        case ELEMENT_OPTION:
            current->current_value ^= 1;
            NXT_setValue(current->object_name, current->current_value);
            if (current->on_change != NULL) {
                current->on_change();
            }
            break;
        case ELEMENT_FLT: // Fall through
        case ELEMENT_VAL:
            page->is_element_selected = true;
            MS_setStyleSelected(current);
            break;
        default:
            break;
    }
}

/**
 * @brief Increments the value of a menu element and updates its display
 * 
 * If incrementing would exceed max_value, wraps around to min_value.
 * Updates display based on element type (float or integer value).
 *
 * @param element Pointer to the menu element to increment
 */
void MS_incrementValue(menu_element_t *element) {
    if (element->current_value + element->increment <= element->max_value) {
        element->current_value += element->increment;
    } else {
        element->current_value = element->min_value;
    }

    switch (element->type) {
        case ELEMENT_FLT:
            NXT_setValue(element->object_name, element->current_value);
            break;
        case ELEMENT_VAL:
            NXT_setTextFormatted(element->object_name, "%d", element->current_value);
            break;
        default:
            break;
    }
}

/**
 * @brief Decrements the value of a menu element and updates its display
 * 
 * If decrementing would exceed min_value, wraps around to max_value.
 * Updates display based on element type (float or integer value).
 *
 * @param element Pointer to the menu element to decrement
 */
void MS_decrementValue(menu_element_t *element) {
    if (element->current_value >= element->increment + element->min_value) {
        element->current_value -= element->increment;
    } else {
        element->current_value = element->max_value;
    }

    switch (element->type) {
        case ELEMENT_FLT:
            NXT_setValue(element->object_name, element->current_value);
            break;
        case ELEMENT_VAL:
            NXT_setTextFormatted(element->object_name, "%d", element->current_value);
            break;
        default:
            break;
    }
}

/**
 * @brief Refreshes the menu page by resetting selection state and updating all elements
 * 
 * @param page Pointer to the menu page to refresh
 */
void MS_refreshPage(menu_page_t *page) {
    page->is_element_selected = false;
    page->current_index = 0;
    int list_index = MS_listGetSelected(page);
    for (uint8_t i = 0; i < page->num_elements; i++) {
        menu_element_t *curr_element = &page->elements[i];
        switch (curr_element->type) {
            case ELEMENT_LIST:
                if (i == list_index) {  // nothing happens if -1
                    MS_setStyleSelected(curr_element);
                }
                break;
            case ELEMENT_VAL:
                NXT_setTextFormatted(curr_element->object_name, "%d", curr_element->current_value);
                break;
            case ELEMENT_FLT: // Fall through
            case ELEMENT_OPTION:
                NXT_setValue(curr_element->object_name, curr_element->current_value);
                break;
            default:
                break;
        }
    }
}

/**
 * @brief Gets the index of the selected element in a menu page
 *
 * @param page Pointer to the menu page structure
 * @return Index of the selected element, -1 if no element is selected
 */
int MS_listGetSelected(menu_page_t *page) {
    for (uint8_t i = 0; i < page->num_elements; i++) {
        if (page->elements[i].current_value) {
            return i;
        }
    }
    return -1;
}