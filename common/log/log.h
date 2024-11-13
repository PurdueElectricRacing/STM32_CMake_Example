/**
 * @file daq_log.c
 * @author Eileen Yoon (eyn@purdue.edu)
 * @brief  vsprintf/snprintf + UART support
 * @version 0.1
 * @date 2024-10-20
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __LOG_H__
#define __LOG_H__

#include "common/common_defs/common_defs.h"
#include "common/log/vsprintf.h"

int snprintf(char *buffer, size_t size, const char *fmt, ...);
int sprintf(char *buffer, const char *fmt, ...);
int debug_printf(const char *fmt, ...);

// cursed macro to workaround accessing different hals
// call DEBUG_PRINTF_USART_DEFINE(&usart_config) ONCE in *main.c* after defining uart config struct
#define DEBUG_PRINTF_USART_DEFINE(HANDLEPTR)                                  \
static inline void _iodev_write(usart_init_t* handle, char *buffer, int size) \
{                                                                             \
    if (handle)                                                               \
    {                                                                         \
        PHAL_usartTxBl(handle, (uint8_t *)buffer, size);                      \
    }                                                                         \
}                                                                             \
                                                                              \
int debug_printf(const char *fmt, ...)                                        \
{                                                                             \
    va_list args;                                                             \
    char buffer[512];                                                         \
    int i;                                                                    \
                                                                              \
    va_start(args, fmt);                                                      \
    i = vsnprintf(buffer, sizeof(buffer), fmt, args);                         \
    va_end(args);                                                             \
                                                                              \
    _iodev_write((HANDLEPTR), buffer, MIN(i, (int)(sizeof(buffer) - 1)));     \
                                                                              \
    return i;                                                                 \
}
// can add SPI etc as necessary

// plog: requires \n
#define plog(...) debug_printf(__VA_ARGS__)

#define ANSI_RED     "\x1b[31m"
#define ANSI_GREEN   "\x1b[32m"
#define ANSI_YELLOW  "\x1b[33m"
#define ANSI_BLUE    "\x1b[34m"
#define ANSI_MAGENTA "\x1b[35m"
#define ANSI_CYAN    "\x1b[36m"
#define ANSI_RESET   "\x1b[37m"

#define __mu_log_color(color, ...) do { \
    debug_printf("%s", color);       \
	debug_printf(__VA_ARGS__);       \
    debug_printf("%s", ANSI_RESET);  \
} while (0)

#define log_msg(...) debug_printf(__VA_ARGS__)
#define log_int(n) (debug_printf("%s == %d\n", (#n), (n)))
#define log_str(s) (debug_printf("%s == %s\n", (#s), (s)))
#define log_char(c) (debug_printf("%s == '%c'\n", (#c), (c)))
#define log_addr(addr) (debug_printf("%s == %p\n", (#addr), (void*)(addr)))
#define log_red(...) __mu_log_color(ANSI_RED, __VA_ARGS__)
#define log_green(...) __mu_log_color(ANSI_GREEN, __VA_ARGS__)
#define log_yellow(...) __mu_log_color(ANSI_YELLOW, __VA_ARGS__)
#define log_blue(...) __mu_log_color(ANSI_BLUE, __VA_ARGS__)
#define log_magenta(...) __mu_log_color(ANSI_MAGENTA, __VA_ARGS__)
#define log_cyan(...) __mu_log_color(ANSI_CYAN, __VA_ARGS__)

#endif // __LOG_H__
