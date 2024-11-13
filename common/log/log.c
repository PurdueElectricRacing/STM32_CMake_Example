
#include "common/common_defs/common_defs.h"
#include "log.h"
#include "vsprintf.h"

int snprintf(char *buffer, size_t size, const char *fmt, ...)
{
    va_list args;
    int i;

    va_start(args, fmt);
    i = vsnprintf(buffer, size, fmt, args);
    va_end(args);
    return i;
}

int sprintf(char *buffer, const char *fmt, ...)
{
    va_list args;
    int i;

    va_start(args, fmt);
    i = vsprintf(buffer, fmt, args);
    va_end(args);

    return i;
}

#if 0
static inline int _iodev_write(usart_init_t* handle, char *buffer, int size)
{
    if (handle)
    {
        PHAL_usartTxBl(handle, (uint16_t *)buffer, size);
    }

    return 0;
}

static inline int _iodev_printf(usart_init_t* handle, const char *fmt, ...)
{
    va_list args;
    char buffer[512];
    int i;

    va_start(args, fmt);
    i = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    _iodev_write(handle, buffer, MIN(i, (int)(sizeof(buffer) - 1)));

    return i;
}
#endif
