#ifndef IMU_FUSION_H
#define IMU_FUSION_H
#include "main.h"

#define ENABLE_IMU_ACC_CALIBRATION	0
#if ENABLE_IMU_ACC_CALIBRATION
	typedef enum {
		CAL_IDLE = 0,
		CAL_WAIT_XP,
		CAL_WAIT_XN,
		CAL_WAIT_YP,
		CAL_WAIT_YN,
		CAL_WAIT_ZP,
		CAL_WAIT_ZN,
		CAL_DONE,
		CAL_FINISH
	} AccCalibState_t;

	typedef struct {
	    float bias_x, bias_y, bias_z;
	    float scale_x, scale_y, scale_z;
	} CalResult_t;
#endif

//after manually acc calibration result
#define ACC_BIAS_X		-0.0104f
#define ACC_BIAS_Y		-0.0177f
#define ACC_BIAS_Z		-0.0103f
#define	ACC_SCALE_X	 	0.9950f
#define ACC_SCALE_Y		0.9980f
#define ACC_SCALE_Z		1.0153f
/*#define ACC_BIAS_X		-0.0015f
#define ACC_BIAS_Y		-0.0181f
#define ACC_BIAS_Z		-0.0126f
#define	ACC_SCALE_X	 	0.9894f
#define ACC_SCALE_Y		0.9986f
#define ACC_SCALE_Z		1.0098f*/

typedef struct {
    float w, x, y, z;
} Quaternion;

typedef struct {
	volatile float gx, gy, gz;   // rad/s
	volatile float ax, ay, az;   // g
	volatile float mx, my, mz;
} IMU_Data_t;

typedef struct
{
	volatile float roll;
	volatile float pitch;
	volatile float yaw;
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
