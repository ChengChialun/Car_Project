#ifndef __MICRO_SECONDS_CONTROL_H
#define __MICRO_SECONDS_CONTROL_H
#include "main.h"
void DWT_Init(void);
void Delay_us(uint32_t us);
uint32_t Get_Us_Timestamp(void);
uint32_t DWT_GetTick(void);
uint32_t DWT_ElapsedUs(uint32_t start);
uint32_t DWT_Now_Us(uint32_t elapsed);
uint64_t Get_SystemTime_Us(void);
#endif
