#ifndef __MOTION_SENSOR_H
#define __MOTION_SENSOR_H
#include "main.h"
#include "MadgwickAHRS.h"
/*********************
*	For ICM20948 DOF
*********************/
#define REG_WHO_AM_I       			0x00
#define REG_USER_CTRL				0x03
#define REG_BANK_SEL 				0x7F
#define ACCEL_XOUT_H 				0x2D
#define PWR_MGMT_1   				0x06
#define PWR_MGMT_2   				0x07
#define REG_INT_PIN_CFG				0x0F
#define REG_INT_ENABLE_1  			0x11
#define REG_INT_STATUS_1  			0x19
#define GYRO_CONFIG_1     			0x01 // Bank 2
#define ACCEL_CONFIG      			0x14 // Bank 2
#define GYRO_SMPLRT_DIV				0x00
#define ACCEL_SMPLRT_DIV_1			0x10
#define ACCEL_SMPLRT_DIV_2			0x11

#define ICM20948_ADDR 				0x68 << 1  // AD0 is low = 0x68, left shift 1 because HAL use 8-bit address
#define AK09916_ADDR 				0x0C << 1

#define ACC_SCALE_2 				16384.0f //2g
#define GRYO_SCALE_250				131.0f   //250 dps

#define MAG_CNT						3		 //1:3
typedef struct{
	int16_t ax, ay, az;//Accel
	int16_t gx, gy, gz;//Gyro
	int16_t Mag_X, Mag_Y, Mag_Z;
}IMU_Data;

typedef enum {
    MAG_IDLE = 200,
    MAG_WAIT_DRDY,
	MAG_READ_DATA
} MAG_STATE;

typedef struct{
	volatile uint8_t INT_Flag;
	volatile uint8_t DMA_Done;
	volatile uint8_t Mag_DMA_Done;
	volatile uint8_t DMA_Busy;
//	volatile MAG_STATE mag_state;
	uint8_t imu_rx_buf[14] __attribute__((aligned(4)));// accel + gyro 14 bytes
	uint8_t mag_raw[9] __attribute__((aligned(4)));//Magnetometer
//	uint8_t mag_raw[6] __attribute__((aligned(4)));//Magnetometer
	uint8_t current_bank;
	uint8_t read_mag_cnt;
	float ax_g, ay_g, az_g;//Accel +-2g
	float gx_dps, gy_dps, gz_dps ;//Gyro +-250dps
	float Mag_X, Mag_Y, Mag_Z;
	float mag_offset_x, mag_offset_y, mag_offset_z;
}IMU;

uint16_t ICM20948_Init();
uint8_t ICM20948_WhoAmI();
void I2C_Motion_Sensor_Init();
uint16_t ICM20948_WriteReg(uint8_t reg, uint8_t data);
uint8_t ICM20948_ReadReg(uint8_t reg);
void ICM20948_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az);
uint16_t ICM20948_Sel_Bank(uint8_t bank);
uint16_t IMU_Read_DMA();
void ICM20948_Parse();
#endif
