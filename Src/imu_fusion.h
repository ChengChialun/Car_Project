#ifndef IMU_FUSION_H
#define IMU_FUSION_H
#include "main.h"

typedef struct {
    float w, x, y, z;
} Quaternion;

typedef struct {
    float gx, gy, gz;   // rad/s
    float ax, ay, az;   // g
    float mx, my, mz;
} IMU_Data_t;

typedef struct
{
	float roll;
	float pitch;
	float yaw;
}Euler_t;

typedef enum {
    CALIB_IDLE = 10,
    CALIB_WAIT_STATIC,
    CALIB_COLLECT,
    CALIB_DONE
} CalibState_t;

void IMU_Fusion_Init();
void IMU_Fusion_Update();
Euler_t IMU_GetEuler();
uint8_t IMU_IsStatic();
void IMU_Calibrate_Task();
void HardIron_Calibrate();
void IMU_Fusion_Update_v2();
void Filter_Quaternion(Quaternion *q_old, Quaternion *q_new, Quaternion *q_out);
#endif
