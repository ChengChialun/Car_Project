#ifndef __POWER_MANAGER_H
#define __POWER_MANAGER_H
#include "main.h"
#include "ADC.h"
#include "Motor.h"
#include "comm.h"

#define LOW_POWER_TIMES 5
extern float voltage;
extern float filtered_voltage;
extern enum power_status PowerStatus;

enum power_status{
	NORMAL_POWER_STATUS = 0,
	PROTECTION_POWER_STATUS,
	STOP_POWER_STATUS
};

float Get_Voltage();
float Get_Power_Limit();
void Power_Manager();
void Power_Manager_Init();
#endif
