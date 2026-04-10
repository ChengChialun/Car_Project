#ifndef __SYSTEM_INTERRUPT_H
#define __SYSTEM_INTERRUPT_H
#include "main.h"
#include "stm32f4xx_hal.h"
#include "Motor.h"

extern volatile uint8_t flag_5ms;
extern volatile uint8_t flag_10ms;
extern volatile uint8_t flag_100ms;
extern volatile uint8_t flag_1s;
#endif