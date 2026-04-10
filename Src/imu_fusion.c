#include "imu_fusion.h"
#include "MadgwickAHRS.h"
#include "math.h"
#include "motion_sensor.h"
#include "debug.h"
// ===== Parameter =====
#define DEG_TO_RAD 		0.01745329251f
#define CALI_SAMPLES 	1000
#define STABLE_CNT 		500
#define GYRO_STATIC_TH 	3.0f   // deg/s = dps  3.0
#define ACC_STATIC_TH 	0.8f   // +- 0.1g	   0.8

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
static float acc_bias_x = 0;
static float acc_bias_y = 0;
static float acc_bias_z = 0;
// ===== time =====
//static float dt = 0.005f;
static float dt = 0.01f;
volatile uint32_t last_tick;
// ===== Initialize =====
#define PI 3.141592653589793f
extern uint8_t is_IMU_init;
extern volatile float beta;
// ===== Quaternion =====
Quaternion q_old = {1, 0, 0, 0};
Quaternion q_filtered = {1, 0, 0, 0};
const float alpha = 0.1f;  //0.05 (stable but latency) ~ 0.15 (fast)

void IMU_Fusion_Init(void)
{
//    last_tick = HAL_GetTick();

    // Madgwick parameter
    sampleFreq = 100.0f;
    beta = 0.41f;
    calib_state = CALIB_WAIT_STATIC;
    ICM20948.mag_offset_x = 0;
    ICM20948.mag_offset_y = 0;
    ICM20948.mag_offset_z = 0;
}

void IMU_Fusion_Update_v2()
{
    // 1. 讀取原始數據並扣除陀螺儀 Bias (陀螺儀 Bias 必須扣除)
    float gx = (ICM20948.gx_dps - gyro_bias_x) * DEG_TO_RAD;
    float gy = (ICM20948.gy_dps - gyro_bias_y) * DEG_TO_RAD;
    float gz = (ICM20948.gz_dps - gyro_bias_z) * DEG_TO_RAD;

    // 2. 加速度計數據 (通常不建議直接減 bias，除非已做 6 面校正)
    float ax = ICM20948.ax_g;
    float ay = ICM20948.ay_g;
    float az = ICM20948.az_g;

    // 3. 加速度計歸一化 (Normalization)
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if(norm > 0) {
        ax /= norm;
        ay /= norm;
        az /= norm;
    }

    // 4. 判斷要跑 9 軸還是 6 軸
    if(ICM20948.Mag_DMA_Done)
    {
    	// 之前你的寫法是 mx=MagY, my=MagX
    	    // 現在請嘗試這組「標準對齊」，它能修正 90 度相位差與正負號衝突
    	  //  float mx_aligned = -(ICM20948.Mag_Y - ICM20948.mag_offset_y); // 注意這裡的負號
    	  //  float my_aligned = (ICM20948.Mag_X - ICM20948.mag_offset_x);
    		float mx_aligned =  (ICM20948.Mag_X - ICM20948.mag_offset_x);
    	      float my_aligned = (ICM20948.Mag_Y - ICM20948.mag_offset_y);
    	    float mz_aligned = -(ICM20948.Mag_Z - ICM20948.mag_offset_z);

    	/*    MadgwickAHRSupdate(imu_data_calibrated.gx, imu_data_calibrated.gy, imu_data_calibrated.gz,
    	                        imu_data_calibrated.ax, imu_data_calibrated.ay, imu_data_calibrated.az,
    	                        mx_aligned, my_aligned, mz_aligned);*/
    	    float yaw_test = atan2f(my_aligned, mx_aligned) * 180.0f / PI;
    	    printf("yaw_test: %.2f\n", yaw_test);
    }
    else
    {
        // 磁力計沒更新時，跑 6 軸確保姿勢不中斷
     //   MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        imu_data_calibrated.ax = ax;
        imu_data_calibrated.ay = ay;
        imu_data_calibrated.az = az;
        imu_data_calibrated.gx = gx;
        imu_data_calibrated.gy = gy;
        imu_data_calibrated.gz = gz;
    }

    // 5. 四元數轉歐拉角
    euler.roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 57.29578f;
    euler.pitch = asinf(2.0f*(q0*q2 - q3*q1)) * 57.29578f;
    euler.yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 57.29578f;

    // 6. Yaw 範圍限制 (-180 ~ 180)
    if (euler.yaw >  180.0f) euler.yaw -= 360.0f;
    if (euler.yaw < -180.0f) euler.yaw += 360.0f;
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
    uint32_t now = HAL_GetTick();
    dt = (now - last_tick) / 1000.0f;
    last_tick = now;

    if (dt <= 0) return;

    sampleFreq = 1.0f / dt;
//    sampleFreq = 100.0f;
    // ===== Gyro Calibration =====
    imu_data_calibrated.gx -= gyro_bias_x;
    imu_data_calibrated.gy -= gyro_bias_y;
    imu_data_calibrated.gz -= gyro_bias_z;

    // ===== Acc Calibration =====
    imu_data_calibrated.ax -= acc_bias_x;
    imu_data_calibrated.ay -= acc_bias_y;
    imu_data_calibrated.az -= acc_bias_z;

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

    return 1; // static
}

void IMU_Calibrate_Task()
{
	static int stable_count = 0;
    static int calib_count = 0;

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
					acc_bias_x = acc_bias_y = acc_bias_z = 0;
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
			acc_bias_x += ICM20948.ax_g;
			acc_bias_y += ICM20948.ay_g;
			acc_bias_z += ICM20948.az_g;
		    calib_count++;

		    if (calib_count >= CALI_SAMPLES)
		    {
		    	gyro_bias_x /= calib_count;
		    	gyro_bias_y /= calib_count;
		    	gyro_bias_z /= calib_count;
		    	acc_bias_x /= calib_count;
		    	acc_bias_y /= calib_count;
		    	acc_bias_z = (acc_bias_z / calib_count) - 1.0f;
/*		    	if(ICM20948.Mag_DMA_Done)
		    		HardIron_Calibrate();*/
		        calib_state = CALIB_DONE;
		        printf("Calibration done!\r\n");
		    }
		break;
    }
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
