#ifndef __STORAGE_CONTROL_H
#define __STORAGE_CONTROL_H
#include "main.h"
#include "micro_seconds_control.h"
#include "string.h"

#include "../../Modules/W25QXX/W25QXX.h"
#define PAYLOAD_SIZE 	66
#define VERSION_LENGTH	4
#define VER_MAJOR 1
#define VER_MINOR 0
#define VER_PATCH 0
#define MAKE_VERSION(major, minor, patch) ( ( (major) << 16) | ( (minor) << 8) | (patch) )
#define VERSION_1_0_0	0x00010000
#define __READ	0
#define __WRITE	1
typedef struct __attribute__((aligned(4))){
    float imu_offset[3]; //accel , gyro, Magnetometer
    float pid_kp;
    float pid_ki;
    float pid_kd;
} Config_Record_t;

typedef struct __attribute__((aligned(4))){
    uint32_t magic;   				// check out 		4 bytes
    uint32_t version;				// config version 	4 bytes
    uint32_t counter;   			// data count		4 bytes
    uint16_t length;   				// data length		2 bytes
    uint8_t  payload[PAYLOAD_SIZE]; // raw data			66 bytes
    uint8_t  padding[(4 - (PAYLOAD_SIZE % 4)) % 4];
    uint32_t checksum;				// CRC				4 bytes
    uint32_t state;     			// EMPTY / WRITING / VALID
} Config_t;

typedef struct __attribute__((aligned(4))){
	uint64_t tick;      // time stamp
    float imu_yaw;
    float imu_pitch;
    float imu_roll;
    float speed;
    float error;
} Log_data_t;

typedef struct __attribute__((aligned(4))){
    uint32_t magic;     // check out
    uint32_t version;
    Log_data_t log_data;
    uint32_t checksum;	// CRC
} Log_t;

void Get_Cur_Conf(Config_t* conf);
Config_Record_t* FLASH_Config_Init(void);
void FLASH_Config_Write(Config_Record_t *data);
#endif

