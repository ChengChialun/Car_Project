#include "imu_fusion.h"
#include "MadgwickAHRS.h"
#include "math.h"
#include "motion_sensor.h"
#include "debug.h"
#include "micro_seconds_control.h"

// ===== Parameter =====
#define DEG_TO_RAD 		0.01745329251f
#define RAD_TO_DEG 		57.29578f		// 360 / (2*Pi)
#define CALI_SAMPLES 	1000
#define STABLE_CNT 		500
#define GYRO_STATIC_TH 	3.0f   			// deg/s = dps  3.0
#define ACC_STATIC_TH 	0.8f   			// +- 0.1g	   0.8

// ===== Manually Calibration =====
#if ENABLE_IMU_ACC_CALIBRATION
	#define CALI_TIMES	5
	#define	ACC_CALI_BUF_SIZE	30
	float ax_buf[ACC_CALI_BUF_SIZE];
	float ay_buf[ACC_CALI_BUF_SIZE];
	float az_buf[ACC_CALI_BUF_SIZE];
	uint16_t buf_idx = 0;
	uint8_t cal_req = 0;
	#define max(a,b) ((a>b)?a:b)
	#define min(a,b) ((a<b)?a:b)
	CalResult_t cal_results[CALI_TIMES];
	extern volatile AccCalibState_t acc_cal_state;
	float acc_avg_x, acc_avg_y, acc_avg_z;
	uint32_t last_cal_tick = 0;
	static float ax_max = -1, ax_min = 1;
	static float ay_max = -1, ay_min = 1;
	static float az_max = -1, az_min = 1;
	void IMU_Calibration_Start(void);
	void Acc_Buffer_Update(float ax, float ay, float az);
	void Acc_Get_Avg(float *ax, float *ay, float *az);
	void IMU_Calibration_Compute(void);
	void Six_Axix_Acc_Calibration(float ax, float ay, float az);
	void Cali_Final_Average();
#endif
// ===== quaternion(Madgwick used)=====
extern volatile float q0, q1, q2, q3;
extern volatile float sampleFreq;
extern IMU ICM20948;
extern volatile IMU_Data_t imu_data_calibrated;
extern volatile CalibState_t calib_state;
// ===== Euler =====
extern volatile Euler_t euler;
// ===== Gyro/Accel bias =====
static float gyro_bias_x = 0;
static float gyro_bias_y = 0;
static float gyro_bias_z = 0;
/*static float acc_bias_x = 0;
static float acc_bias_y = 0;
static float acc_bias_z = 0;*/
// ===== time =====
//static float dt = 0.005f;
static float dt = 0.01f;
volatile uint32_t last_tick;
// ===== Initialize =====
#define PI 3.141592653589793f
extern uint8_t is_IMU_init;
extern volatile float beta;
volatile float dt_avg;
// ===== Quaternion =====
Quaternion q_old = {1, 0, 0, 0};
Quaternion q_filtered = {1, 0, 0, 0};
const float alpha = 0.1f;  //0.05 (stable but latency) ~ 0.15 (fast)

void IMU_Fusion_Init(void)
{
    last_tick = DWT_GetTick();

    // Madgwick parameter
    sampleFreq = 100.0f;
    beta = betaDef;
    calib_state = CALIB_WAIT_STATIC;
    ICM20948.mag_offset_x = 0;
    ICM20948.mag_offset_y = 0;
    ICM20948.mag_offset_z = 0;
}

void IMU_Fusion_Update()
{
	imu_data_calibrated.ax = ICM20948.ax_g;
	imu_data_calibrated.ay = ICM20948.ay_g;
	imu_data_calibrated.az = ICM20948.az_g;
	imu_data_calibrated.gx = ICM20948.gx_dps;
	imu_data_calibrated.gy = ICM20948.gy_dps;
	imu_data_calibrated.gz = ICM20948.gz_dps;

    // ===== caculate dt =====
	uint32_t now = DWT_GetTick();
	dt = (now - last_tick) / (float)SystemCoreClock;
	last_tick = now;
	dt_avg = 0.9f * dt_avg + 0.1f * dt;
    sampleFreq = 1.0f / dt_avg;

    //    sampleFreq = 100.0f;
    // ===== Gyro Calibration =====
    imu_data_calibrated.gx -= gyro_bias_x;
    imu_data_calibrated.gy -= gyro_bias_y;
    imu_data_calibrated.gz -= gyro_bias_z;

    // ===== Acc Calibration =====
    imu_data_calibrated.ax = (imu_data_calibrated.ax - ACC_BIAS_X) / ACC_SCALE_X;
    imu_data_calibrated.ay = (imu_data_calibrated.ay - ACC_BIAS_Y) / ACC_SCALE_Y;
    imu_data_calibrated.az = (imu_data_calibrated.az - ACC_BIAS_Z) / ACC_SCALE_Z;

    // ===== deg/s to rad/s)=====
    imu_data_calibrated.gx *= DEG_TO_RAD;
    imu_data_calibrated.gy *= DEG_TO_RAD;
    imu_data_calibrated.gz *= DEG_TO_RAD;

    // ===== Acc normalize =====
    float norm = sqrtf(imu_data_calibrated.ax*imu_data_calibrated.ax + imu_data_calibrated.ay*imu_data_calibrated.ay + imu_data_calibrated.az*imu_data_calibrated.az);
    if(norm == 0) return;

    imu_data_calibrated.ax /= norm;
    imu_data_calibrated.ay /= norm;
    imu_data_calibrated.az /= norm;

    float gyro_norm = fabs(imu_data_calibrated.gx) + fabs(imu_data_calibrated.gy) + fabs(imu_data_calibrated.gz);

    if (gyro_norm > 5.0f) {
    	beta = 0.01f;
    } else {
    	beta = betaDef;  // static
    }
    // ===== Madgwick =====
    if(ICM20948.Mag_DMA_Done)
    {
    	float tmp;
    	imu_data_calibrated.mx = ICM20948.Mag_X;
    	imu_data_calibrated.my = ICM20948.Mag_Y;
    	imu_data_calibrated.mz = ICM20948.Mag_Z;
    	// ===== Mag Calibration =====
    	imu_data_calibrated.mx -= ICM20948.mag_offset_x;
    	imu_data_calibrated.my -= ICM20948.mag_offset_y;
    	imu_data_calibrated.mz -= ICM20948.mag_offset_z;
    	// ===== Mag Normalization =====
    	float mag_norm = sqrtf( imu_data_calibrated.mx*imu_data_calibrated.mx +
    							imu_data_calibrated.my*imu_data_calibrated.my +
								imu_data_calibrated.mz*imu_data_calibrated.mz);
    	if(mag_norm > 0)
    	{
    		imu_data_calibrated.mx /= mag_norm;
    		imu_data_calibrated.my /= mag_norm;
    		imu_data_calibrated.mz /= mag_norm;
    	}
    	tmp = imu_data_calibrated.mx;
    	imu_data_calibrated.mx = imu_data_calibrated.my;
    	imu_data_calibrated.my = tmp;
    	imu_data_calibrated.mz = -imu_data_calibrated.mz;
    	MadgwickAHRSupdate( imu_data_calibrated.gx, imu_data_calibrated.gy, imu_data_calibrated.gz,
    						imu_data_calibrated.ax, imu_data_calibrated.ay, imu_data_calibrated.az,
							imu_data_calibrated.mx, imu_data_calibrated.my, imu_data_calibrated.mz
							);
    }
    else
    	MadgwickAHRSupdateIMU(imu_data_calibrated.gx, imu_data_calibrated.gy, imu_data_calibrated.gz, imu_data_calibrated.ax, imu_data_calibrated.ay, imu_data_calibrated.az);

    Quaternion q_new = {q0, q1, q2, q3};
	if( is_IMU_init ) {
		q_old = q_new;
		q_filtered = q_new;
		is_IMU_init = 0;
	}

	//sign flip
	float dot = q_old.w*q_new.w + q_old.x*q_new.x + q_old.y*q_new.y + q_old.z*q_new.z;
	if (dot < 0.0f) {
		q_new.w = -q_new.w;
		q_new.x = -q_new.x;
		q_new.y = -q_new.y;
		q_new.z = -q_new.z;
	}

	Filter_Quaternion(&q_old, &q_new, &q_filtered);
	q_old = q_filtered;
    // ===== Quaternion to Euler =====
    euler.roll = atan2f(2.0f*(q_filtered.w*q_filtered.x + q_filtered.y*q_filtered.z),
                        1 - 2*(q_filtered.x*q_filtered.x + q_filtered.y*q_filtered.y)) * 57.29578f;

    euler.pitch = asinf(2.0f*(q_filtered.w*q_filtered.y - q_filtered.z*q_filtered.x)) * 57.29578f;

    //-180 ~ +180
    euler.yaw = atan2f(2.0f*(q_filtered.w*q_filtered.z + q_filtered.x*q_filtered.y), 1.0f - 2.0f*(q_filtered.y*q_filtered.y + q_filtered.z*q_filtered.z)) * 57.29578f;
    while (euler.yaw >  180.0f) euler.yaw -= 360.0f;
    while (euler.yaw < -180.0f) euler.yaw += 360.0f;
}

Euler_t IMU_GetEuler(void)
{
    return euler;
}

uint8_t IMU_IsStatic()
{
    // Gyro
    if (fabs(ICM20948.gx_dps) > GYRO_STATIC_TH ||
        fabs(ICM20948.gy_dps) > GYRO_STATIC_TH ||
        fabs(ICM20948.gz_dps) > GYRO_STATIC_TH)
        return 0;

    // Acc magnitude
    float acc_norm = sqrtf(ICM20948.ax_g*ICM20948.ax_g + ICM20948.ay_g*ICM20948.ay_g + ICM20948.az_g*ICM20948.az_g);

    if (fabs(acc_norm - 1.0f) > ACC_STATIC_TH)
        return 0;
#if (!ENABLE_IMU_ACC_CALIBRATION)
    if(ICM20948.az_g / acc_norm <= 0.96f) // check z+
    	return 0;
#endif
    return 1; // static
}

void IMU_Calibrate_Task()
{
	static int stable_count = 0;
    static int calib_count = 0;

#if (!ENABLE_IMU_ACC_CALIBRATION)
    switch(calib_state)
    {
		case CALIB_WAIT_STATIC:
			if (IMU_IsStatic())
			{
				stable_count++;
				if(stable_count > STABLE_CNT)
				{
					printf("Static detected. Calibrating...\r\n");
					stable_count = 0;
					gyro_bias_x = gyro_bias_y = gyro_bias_z = 0;
//					acc_bias_x = acc_bias_y = acc_bias_z = 0;
					calib_state = CALIB_COLLECT;
				}
			}
			else
				stable_count = 0;
        break;
		case CALIB_COLLECT:
			gyro_bias_x += ICM20948.gx_dps;
			gyro_bias_y += ICM20948.gy_dps;
			gyro_bias_z += ICM20948.gz_dps;
		/*
			acc_bias_x += ICM20948.ax_g;
			acc_bias_y += ICM20948.ay_g;
			acc_bias_z += ICM20948.az_g;
			*/
		    calib_count++;

		    if (calib_count >= CALI_SAMPLES)
		    {
		    	gyro_bias_x /= calib_count;
		    	gyro_bias_y /= calib_count;
		    	gyro_bias_z /= calib_count;
		    	/*
		    	acc_bias_x /= calib_count;
		    	acc_bias_y /= calib_count;
		    	acc_bias_z = (acc_bias_z / calib_count) - 1.0f;
		    	*/
/*		    	if(ICM20948.Mag_DMA_Done)
		    		HardIron_Calibrate();*/
		        calib_state = CALIB_DONE;
		        printf("Calibration done!\r\n");
		    }
		break;
    }
#else
    if( cal_req < CALI_TIMES)
    {
    	if(IMU_IsStatic())
    	{
    		stable_count++;
    		if(stable_count > STABLE_CNT)
    		{
    			Acc_Buffer_Update(ICM20948.ax_g, ICM20948.ay_g, ICM20948.az_g);
    			if(HAL_GetTick() - last_cal_tick > 50) // 20Hz
    			{
    				last_cal_tick = HAL_GetTick();
    				Acc_Get_Avg(&acc_avg_x, &acc_avg_y, &acc_avg_z);
    				if(acc_cal_state == CAL_DONE)
    					IMU_Calibration_Start();
    				Six_Axix_Acc_Calibration(acc_avg_x, acc_avg_y, acc_avg_z);
    			}
    		}
    	}
    	else stable_count = 0;
    }
    else if( cal_req >= CALI_TIMES && acc_cal_state != CAL_FINISH )
    	Cali_Final_Average();
#endif
}

float mag_x_min=9999, mag_x_max=-9999;
float mag_y_min=9999, mag_y_max=-9999;
float mag_z_min=9999, mag_z_max=-9999;

void HardIron_Calibrate() {
    if(ICM20948.Mag_X < mag_x_min) mag_x_min = ICM20948.Mag_X;
    if(ICM20948.Mag_X > mag_x_max) mag_x_max = ICM20948.Mag_X;
    if(ICM20948.Mag_Y < mag_y_min) mag_y_min = ICM20948.Mag_Y;
    if(ICM20948.Mag_Y > mag_y_max) mag_y_max = ICM20948.Mag_Y;
    if(ICM20948.Mag_Z < mag_z_min) mag_z_min = ICM20948.Mag_Z;
    if(ICM20948.Mag_Z > mag_z_max) mag_z_max = ICM20948.Mag_Z;

    ICM20948.mag_offset_x = (mag_x_max + mag_x_min) / 2.0f;
    ICM20948.mag_offset_y = (mag_y_max + mag_y_min) / 2.0f;
    ICM20948.mag_offset_z = (mag_z_max + mag_z_min) / 2.0f;
}

//Quaternion Spatial Filtering
void Filter_Quaternion(Quaternion *q_old, Quaternion *q_new, Quaternion *q_out)
{
    // Low-pass filter
    q_out->w = (1 - alpha) * q_old->w + alpha * q_new->w;
    q_out->x = (1 - alpha) * q_old->x + alpha * q_new->x;
    q_out->y = (1 - alpha) * q_old->y + alpha * q_new->y;
    q_out->z = (1 - alpha) * q_old->z + alpha * q_new->z;

    // normalization
    float norm = sqrtf(q_out->w*q_out->w + q_out->x*q_out->x +
                       q_out->y*q_out->y + q_out->z*q_out->z);

    q_out->w /= norm;
    q_out->x /= norm;
    q_out->y /= norm;
    q_out->z /= norm;

    //update
    q_old = q_out;
}

#if ENABLE_IMU_ACC_CALIBRATION

void IMU_Calibration_Start(void)
{
	acc_cal_state = CAL_WAIT_XP;

    ax_max = ay_max = az_max = -1;
    ax_min = ay_min = az_min =  1;

    printf("IMU CAL START\r\n");
    printf("Place +X UP\r\n");
}

void Acc_Buffer_Update(float ax, float ay, float az)
{
    ax_buf[buf_idx] = ax;
    ay_buf[buf_idx] = ay;
    az_buf[buf_idx] = az;

    buf_idx++;
    if(buf_idx >= ACC_CALI_BUF_SIZE)
        buf_idx = 0;
}

void Acc_Get_Avg(float *ax, float *ay, float *az)
{
    *ax = 0; *ay = 0; *az = 0;

    for(int i = 0; i < ACC_CALI_BUF_SIZE; i++)
    {
        *ax += ax_buf[i];
        *ay += ay_buf[i];
        *az += az_buf[i];
    }

    *ax /= ACC_CALI_BUF_SIZE;
    *ay /= ACC_CALI_BUF_SIZE;
    *az /= ACC_CALI_BUF_SIZE;
}

void Six_Axix_Acc_Calibration(float ax, float ay, float az)
{
	if(acc_cal_state == CAL_IDLE || acc_cal_state == CAL_DONE || acc_cal_state == CAL_FINISH)
		return;

	float nx, ny, nz;
	float norm = sqrtf(ax*ax + ay*ay + az*az);
	nx = ax / norm;
	ny = ay / norm;
	nz = az / norm;

	switch(acc_cal_state)
	{
		case CAL_WAIT_XP:
			if(nx > 0.96f)
			{
				ax_max = max(ax_max, ax);
				ax_min = min(ax_min, ax);

				printf("XP captured\r\n");
				acc_cal_state = CAL_WAIT_XN;
				printf("Place -X UP\r\n");
			}
			break;

		case CAL_WAIT_XN:
			if(nx < -0.96f)
			{
				ax_max = max(ax_max, ax);
				ax_min = min(ax_min, ax);

				printf("XN captured\r\n");
				acc_cal_state = CAL_WAIT_YP;
				printf("Place +Y UP\r\n");
			}
			break;

		case CAL_WAIT_YP:
			if(ny > 0.96f)
			{
				ay_max = max(ay_max, ay);
				ay_min = min(ay_min, ay);

				printf("YP captured\r\n");
				acc_cal_state = CAL_WAIT_YN;
				printf("Place -Y UP\r\n");
			}
			break;

		case CAL_WAIT_YN:
			if(ny < -0.96f)
			{
				ay_max = max(ay_max, ay);
				ay_min = min(ay_min, ay);

				printf("YN captured\r\n");
				acc_cal_state = CAL_WAIT_ZP;
				printf("Place +Z UP\r\n");
			}
			break;

		case CAL_WAIT_ZP:
			if(nz > 0.96f)
			{
				az_max = max(az_max, az);
				az_min = min(az_min, az);

				printf("ZP captured\r\n");
				acc_cal_state = CAL_WAIT_ZN;
	            printf("Place -Z UP\r\n");
	        }
	        break;

		case CAL_WAIT_ZN:
			if(nz < -0.96f)
			{
				az_max = max(az_max, az);
				az_min = min(az_min, az);

				printf("ZN captured\r\n");
				acc_cal_state = CAL_DONE;
				IMU_Calibration_Compute();
			}
			break;

		default:
			break;
	}
}

void IMU_Calibration_Compute(void)
{
	float bias_x = (ax_max + ax_min) * 0.5f;
	float bias_y = (ay_max + ay_min) * 0.5f;
	float bias_z = (az_max + az_min) * 0.5f;

	float scale_x = (ax_max - ax_min) * 0.5f;
	float scale_y = (ay_max - ay_min) * 0.5f;
	float scale_z = (az_max - az_min) * 0.5f;

	printf("=== CAL RESULT ===\r\n");
	printf("bias: %.4f %.4f %.4f\r\n", bias_x, bias_y, bias_z);
	printf("scale: %.4f %.4f %.4f\r\n", scale_x, scale_y, scale_z);

    cal_results[cal_req].bias_x = bias_x;
    cal_results[cal_req].bias_y = bias_y;
    cal_results[cal_req].bias_z = bias_z;
    cal_results[cal_req].scale_x = scale_x;
    cal_results[cal_req].scale_y = scale_y;
    cal_results[cal_req].scale_z = scale_z;
	cal_req++;
}

void Cali_Final_Average()
{
	float bx=0, by=0, bz=0;
	float sx=0, sy=0, sz=0;

	for(int i=0;i<CALI_TIMES;i++)
	{
	    bx += cal_results[i].bias_x;
	    by += cal_results[i].bias_y;
	    bz += cal_results[i].bias_z;

	    sx += cal_results[i].scale_x;
	    sy += cal_results[i].scale_y;
	    sz += cal_results[i].scale_z;
	}

	bx /= CALI_TIMES;
	by /= CALI_TIMES;
	bz /= CALI_TIMES;

	sx /= CALI_TIMES;
	sy /= CALI_TIMES;
	sz /= CALI_TIMES;

	printf("=== FINAL RESULT ===\r\n");
	printf("bias: %.4f %.4f %.4f\r\n", bx, by, bz);
	printf("scale: %.4f %.4f %.4f\r\n", sx, sy, sz);
	acc_cal_state = CAL_FINISH;
}
#endif
