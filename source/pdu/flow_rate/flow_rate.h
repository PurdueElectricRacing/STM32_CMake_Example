#ifndef _FAN_CONTROL_H_
#define _FAN_CONTROL_H_

#include <stdbool.h>
#include "stm32f407xx.h"
#include "main.h"

bool flowRateInit();
uint32_t getFlowRate1();
uint32_t getFlowRate2();

#endif // _FAN_CONTROL_H_
