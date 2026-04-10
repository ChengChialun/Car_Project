#ifndef __DEBUG_H
#define __DEBUG_H

//This debug function are working on uart1
#include "stdio.h"
#include "usart.h"
#include "stm32f4xx_hal_uart.h"

#define DEBUG_MODE 1
#define ENABLE_IMU 0
int fputc(int ch, FILE *f);
void Debug_Unfreeze();
void Set_Run_On_Ram();
#endif
