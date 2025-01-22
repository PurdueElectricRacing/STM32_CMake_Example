/**
 * @file buffer.c
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Buffer for one producer, multiple consumers
 * @version 0.1
 * @date 2024-02-18
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "buffer.h"

static void bUpdateMinTail(b_handle_t *b);

void bConstruct(b_handle_t *b, uint32_t item_size, uint32_t buff_size)
{
    b->_item_size = item_size;
    b->_max_items = (buff_size / (float) item_size);
    b->_head = 0;
    b->_min_tail = 0;
    b->overflows = 0;
    b->underflows = 0;
    b->max_level = 0;
    b->_min_tail_active = false;
    for (uint8_t i = 0; i < b->num_tails; ++i)
    {
        b->tails[i]._tail = 0;
        b->tails[i].active = false;
    }
}

/**
 * @brief Gets the head location for writing 1 item
 *        Call bCommitWrite when done writing data
 * @param b
 * @param tx_loc
 * @param available
 * @return uint8_t
 */
int8_t bGetHeadForWrite(b_handle_t *b, void** tx_loc, uint32_t *contiguous_items)
{
    uint32_t head = b->_head;
    if (!b->_min_tail_active) *contiguous_items = b->_max_items - head;
    else if (head < b->_min_tail)
    {
        *contiguous_items = (b->_min_tail - 1) - head;
    }
    else
    {
        *contiguous_items = b->_max_items - head;
        if (b->_min_tail == 0) (*contiguous_items)--; // edge case
    }
    if (*contiguous_items == 0)
    {
        ++b->overflows;
        return -1; // No room
    }
    *tx_loc = (uint8_t *)b->buffer + (head * b->_item_size);
    return 0;
}

/**
 * @brief Updates the head by 1 item
 *        Call after successful bGetHeadForWrite
 *        and AFTER writing your data
 *        Note: Ensure these calls occur in order
 *              (i.e. not preempted by another producer)
 * @param b
 * @return uint8_t
 */
int8_t bCommitWrite(b_handle_t *b, uint32_t items_written)
{
    uint32_t head = b->_head;
    uint32_t max_contig;
    if (!b->_min_tail_active) max_contig = b->_max_items - head;
    else if (head < b->_min_tail)
        max_contig = (b->_min_tail - 1) - head;
    else
    {
        max_contig = b->_max_items - head;
        if (b->_min_tail == 0) max_contig--; // edge case
    }
    if (items_written > max_contig )
    {
        ++b->overflows;
        return -1;
    }
    uint32_t next_head = (head + items_written) % b->_max_items;
    b->_head = next_head;
    uint32_t level = bGetFillLevel(b);
    if (level > b->max_level) b->max_level = level;
    return 0;
}

int8_t bSendToBack(b_handle_t *b, const void* data)
{
    uint32_t next_head = (b->_head + 1) % b->_max_items;
    if (b->_min_tail_active && next_head == b->_min_tail)
    {
        ++b->overflows;
        return -1;                // No room
    }
    volatile uint8_t* dst = b->buffer + (b->_head * b->_item_size);
    for (uint32_t i = 0; i < b->_item_size; ++i) dst[i] = ((uint8_t *)data)[i];
    b->_head = next_head;                                    // Update head
    return 0;
}

static void bUpdateMinTail(b_handle_t *b)
{
    uint32_t max_to_read = 0;
    uint32_t curr_to_read;
    uint32_t head = b->_head; // Capture head (volatile)
    uint32_t next_min_tail = head;
    b_tail_t *t = b->tails;
    bool     tail_active = false;
    // If no tails active, updates to head (empties buffer)
    // Finds the active tail that is furthest from the head
    for (uint8_t i = 0; i < b->num_tails; ++i)
    {
        if (t->active)
        {
            tail_active = true;
            if (t->_tail <= head) curr_to_read = head - t->_tail;
            else curr_to_read = b->_max_items - (t->_tail - head);
            if (curr_to_read > max_to_read)
            {
                max_to_read = curr_to_read;
                next_min_tail = t->_tail;
            }
        }
        ++t;
    }
    b->_min_tail = next_min_tail;
    b->_min_tail_active = tail_active;
    // Set tail then if its active
    // When reading, read if active first, then the value
}

/**
 * @brief Returns number of items left to read for given tail
 *
 */
uint32_t bGetItemCount(b_handle_t *b, uint8_t t_idx)
{
    b_tail_t *t = b->tails + t_idx;
    if (!t->active) return 0;
    uint32_t head = b->_head;
    if (t->_tail <= head) return head - t->_tail;
    else return b->_max_items - (t->_tail - head);
}

/**
 * @brief Returns the total number of items in the buffer
 *
 */
uint32_t bGetFillLevel(b_handle_t *b)
{
    uint32_t head = b->_head;
    if (!b->_min_tail_active) return 0;
    if (b->_min_tail <= head) return head - b->_min_tail;
    else return b->_max_items - (b->_min_tail - head);
}

void bActivateTail(b_handle_t *b, uint8_t t_idx)
{
    b_tail_t *t = b->tails + t_idx;
    t->_tail = b->_head; // Initialize to be caught up
    t->active = true;
    bUpdateMinTail(b);
}

void bDeactivateTail(b_handle_t *b, uint8_t t_idx)
{
    b->tails[t_idx].active = false;
    bUpdateMinTail(b);
}

/**
 * @brief Gets the number of contiguous items available to read at *rx_loc
 *
 * @param b
 * @param tail
 * @param rx_loc
 * @param contiguous_items
 * @return int8_t
 */
int8_t bGetTailForRead(b_handle_t *b, uint8_t t_idx, void** rx_loc, uint32_t *contiguous_items)
{
    b_tail_t *t = b->tails + t_idx;
    if (!t->active) return -1; // Gotcha
    uint32_t head = b->_head; // Capture current head (volatile)
    if (t->_tail <= head) *contiguous_items = head - t->_tail;
    else *contiguous_items = b->_max_items - t->_tail;
    if (*contiguous_items == 0) return -1;
    *rx_loc = (uint8_t *)b->buffer + ((uint32_t) t->_tail * b->_item_size);
    return 0;
}

/**
 * @brief Updates the given tail by items_read
 *        Call after successful bGetTailForRead
 *        and AFTER reading your data.
 *        Note: ensure read calls of a tail occur in order
 *        Note: ensure 'items_read' is not larger than
 *              the contiguous_items that was given
 *
 * @param b
 * @param t
 * @param items_read Number of items read (NOT bytes)
 * @return int8_t
 */
int8_t bCommitRead(b_handle_t *b, uint8_t t_idx, uint32_t items_read)
{
    b_tail_t *t = b->tails + t_idx;
    if (!t->active) return -1; // Gotcha
    // Verify number of items read
    uint32_t head = b->_head;
    uint32_t max_contig_items = 0;
    if (t->_tail <= head) max_contig_items = head - t->_tail;
    else max_contig_items = b->_max_items - t->_tail;
    if (items_read > max_contig_items)
    {
        ++b->underflows;
        return -2;
    }
    t->_tail = (t->_tail + items_read) % b->_max_items;
    bUpdateMinTail(b);
    return 0;
}
