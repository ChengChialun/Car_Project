#ifndef __ULTRASONIC_SENSOR_H
#define __ULTRASONIC_SENSOR_H
#include <micro_seconds_control.h>
#include "main.h"
#include <stdint.h>
#include <string.h>
#include "debug.h"
#include "usart.h"

#define HISTORY_DATA_SIZE		5
#define US100_ENABLED			1
#define US100_RX_LENGTH			2
#define US100_CMD_TRIGER		0x55
#define FRONT_US_UART_HANDLE	huart5
#define REAR_US_UART_HANDLE		huart4
#define INVALID_DISTANCE   		0.0f
#if US100_ENABLED
	#define US_HTIM_HANDLE		NULL
#else
	#define US_HTIM_HANDLE		&(htim5)
#endif
typedef enum {
    TRIG_FRONT,
    WAIT_FRONT,
	INTERVAL_DELAY_REAR,
    TRIG_REAR,
    WAIT_REAR,
	INTERVAL_DELAY_FRONT
} SonicState_t;

typedef struct{
#if US100_ENABLED
	uint16_t raw_mm;
	uint8_t rx_buf[US100_RX_LENGTH];
#else
	uint32_t start, end;
#endif
	uint8_t start_detect, finish_detect;
	float distance;
	float history_data[HISTORY_DATA_SIZE];
	uint8_t idx;	//ring buffer point
}Ultrasonic_t;

extern volatile SonicState_t sonic_state;
uint32_t Ultrasonic_Init(TIM_HandleTypeDef *htim);
void Ultrasonic_Task(TIM_HandleTypeDef *htim);
#endif
