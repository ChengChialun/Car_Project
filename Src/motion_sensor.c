#include "motion_sensor.h"
#include "stm32f4xx_hal.h"
#include "debug.h"
/*********************
*	 For ICM20948 DOF
|  Bank		| Purpose  |
| --------| -------- |
|	BANK 0	|	Base data, Initialize(Accel / Gyro / WHO_AM_I)
|	BANK 1	|	Sensor setting
|	BANK 2	|	Gyro / Accel setting
|	BANK 3	|	I2C Master(Internal)
*********************/

extern IMU ICM20948;
extern I2C_HandleTypeDef hi2c1;
static float acc_scale;
static float gyro_scale;
extern uint8_t is_IMU_init;
// ICM20948 Write Register
uint16_t ICM20948_WriteReg(uint8_t reg, uint8_t data){
	if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100)){
		return ERROR_I2C;
	}
	else return OK;
}

// ICM20948 Read Register
uint8_t ICM20948_ReadReg(uint8_t reg){
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return data;
}

// ICM20948 Select Bank
uint16_t ICM20948_Sel_Bank(uint8_t bank){
	if(ICM20948.current_bank == bank) return OK;
	uint8_t data = (bank << 4);  // bank put on bit[5:4]
	if(HAL_I2C_Mem_Write(&hi2c1,
                     ICM20948_ADDR,
                     REG_BANK_SEL,   // 0x7F
                     I2C_MEMADD_SIZE_8BIT,
                     &data,
                     1,
                     100))
		return ERROR_IMU_BANK_SEL;
	HAL_Delay(2);
	ICM20948.current_bank = bank;

	return OK;
}

//Read from IMU
void ICM20948_Read_All(void)
{
	ICM20948_Sel_Bank(0);

	HAL_I2C_Mem_Read(&hi2c1,
                    ICM20948_ADDR,
                    0x2D,  // ACCEL_XOUT_H
                    I2C_MEMADD_SIZE_8BIT,
                    ICM20948.imu_rx_buf,
                    14,
                    100);
}

/*  void AK09916_Update(void) {
  switch(ICM20948.mag_state) {
        case MAG_IDLE:
            // read ST1 (0x10)
        	ICM20948.mag_state = MAG_WAIT_DRDY;
            HAL_I2C_Mem_Read_DMA(&hi2c1, AK09916_ADDR, 0x10, I2C_MEMADD_SIZE_8BIT, ICM20948.mag_raw, 1);
            break;

        case MAG_WAIT_DRDY:
        	if(ICM20948.mag_raw[0] & 0x01) { // DRDY = 1
        		// read 6 byte Mag data (X,Y,Z)
        		HAL_I2C_Mem_Read_DMA(&hi2c1, AK09916_ADDR, 0x11, I2C_MEMADD_SIZE_8BIT, ICM20948.mag_raw, 6);
        		ICM20948.mag_state = MAG_READ_DATA;
        	}
        	else {
        		//ICM20948.DMA_Busy = 0;  // DRDY not ready，dont do anything
        		ICM20948.mag_state = MAG_IDLE;
        	}
            break;
    }
    ICM20948.DMA_Busy = 0;
}*/

//Reads an amount of data in non-blocking mode with DMA
uint16_t IMU_Read_DMA(void)
{	
	if (hi2c1.State != HAL_I2C_STATE_READY) return ERROR_I2C_BUSY;
//	if (ICM20948.DMA_Busy) return ERROR_I2C_BUSY;

	ICM20948_Sel_Bank(0);
	//Read INT status  (Bank 0, 0x19)
	//Access I2C DMA
	ICM20948.DMA_Busy = 1; //locked
	ICM20948.read_mag_cnt++;
	if(ICM20948.read_mag_cnt > MAG_CNT)
	{
		if(HAL_I2C_Mem_Read_DMA(&hi2c1, AK09916_ADDR, 0x10, I2C_MEMADD_SIZE_8BIT, ICM20948.mag_raw, 9) != HAL_OK ) {
			ICM20948.DMA_Busy = 0; //unlock
		}
		//AK09916_Update();
	}
	else
	{
		if(HAL_I2C_Mem_Read_DMA(&hi2c1,
								ICM20948_ADDR,
								0x2D, // Accel Register
								I2C_MEMADD_SIZE_8BIT,
								ICM20948.imu_rx_buf,
								14) != HAL_OK)
		{
			ICM20948.DMA_Busy = 0; //unlock
		}
	}
	
	return OK;
}

uint8_t ICM20948_WhoAmI()
{
	uint8_t whoami = 0;

	ICM20948_Sel_Bank(0);
	HAL_I2C_Mem_Read(&hi2c1,
                   ICM20948_ADDR,
                   REG_WHO_AM_I,
                   I2C_MEMADD_SIZE_8BIT,
                   &whoami,
                   1,
                   100);
#if DEBUG_MODE
	printf("whoami: %d\r\n",whoami);
#endif
	return whoami;
}

void ICM20948_Parse()
{
	if(ICM20948.Mag_DMA_Done){
		// AK09916 0.15 uT/LSB
		if ( (ICM20948.mag_raw[0] & 0x01)  && !(ICM20948.mag_raw[8] & 0x08) ) { //check st1 and st2
			ICM20948.Mag_X = (int16_t)(ICM20948.mag_raw[2]<<8 | ICM20948.mag_raw[1]) * 0.15f;
			ICM20948.Mag_Y = (int16_t)(ICM20948.mag_raw[4]<<8 | ICM20948.mag_raw[3]) * 0.15f;
			ICM20948.Mag_Z = (int16_t)(ICM20948.mag_raw[6]<<8 | ICM20948.mag_raw[5]) * 0.15f;
		}
		/*ICM20948.Mag_X = (int16_t)(ICM20948.mag_raw[1] << 8 | ICM20948.mag_raw[0]) * 0.15f;
		ICM20948.Mag_Y = (int16_t)(ICM20948.mag_raw[3] << 8 | ICM20948.mag_raw[2]) * 0.15f;
		ICM20948.Mag_Z = (int16_t)(ICM20948.mag_raw[5] << 8 | ICM20948.mag_raw[4]) * 0.15f;*/
		//Hard Iron
		ICM20948.Mag_X -= ICM20948.mag_offset_x;
		ICM20948.Mag_Y -= ICM20948.mag_offset_y;
		ICM20948.Mag_Z -= ICM20948.mag_offset_z;

//		ICM20948.mag_state = MAG_IDLE;
	}
	if(ICM20948.DMA_Done)
	{
		IMU_Data raw_data;
		raw_data.ax = (ICM20948.imu_rx_buf[0] << 8) | ICM20948.imu_rx_buf[1];
		raw_data.ay = (ICM20948.imu_rx_buf[2] << 8) | ICM20948.imu_rx_buf[3];
		raw_data.az = (ICM20948.imu_rx_buf[4] << 8) | ICM20948.imu_rx_buf[5];

		raw_data.gx = (ICM20948.imu_rx_buf[6] << 8) | ICM20948.imu_rx_buf[7];
		raw_data.gy = (ICM20948.imu_rx_buf[8] << 8) | ICM20948.imu_rx_buf[9];
		raw_data.gz = (ICM20948.imu_rx_buf[10] << 8) | ICM20948.imu_rx_buf[11];
	
		ICM20948.ax_g = raw_data.ax * acc_scale;
		ICM20948.ay_g = raw_data.ay * acc_scale;
		ICM20948.az_g = raw_data.az * acc_scale;
		ICM20948.gx_dps = raw_data.gx * gyro_scale;
		ICM20948.gy_dps = raw_data.gy * gyro_scale;
		ICM20948.gz_dps = raw_data.gz * gyro_scale;
	}
}

uint16_t ICM20948_Init()
{
	// Reset (ICM20948_Sel_Bank(0))
	uint8_t data = (0 << 4);
	HAL_I2C_Mem_Write(&hi2c1,
                     ICM20948_ADDR,
                     REG_BANK_SEL,   // 0x7F
                     I2C_MEMADD_SIZE_8BIT,
                     &data,
                     1,
                     100);
	ICM20948.current_bank = 0;

	//PWR_MGMT_1 reset
	data = 0x80;
	if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100)) {
		return ERROR_IMU_RESET;
	}
//  ICM20948_WriteReg(PWR_MGMT_1, 0x80);
	HAL_Delay(150);
	uint8_t whoami_cnt= 0;
	do{
		HAL_Delay(10);
		if(ICM20948_WhoAmI()) whoami_cnt++;
	}while(whoami_cnt < 5);

  // Wake up & Auto clock
//  ICM20948_WriteReg(PWR_MGMT_1, 0x01);
	data = 0x01;
	if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100)) {
		return ERROR_IMU_WAKEUP;
	}
	HAL_Delay(20);

	// Set Gyro / Accel
	if(ICM20948_Sel_Bank(2))
		return ERROR_IMU_BANK_SEL;

	// Gyro: +-250 dps , DLPF enable
	data = (0x00 << 1) | 0x01;
	if(ICM20948_WriteReg(GYRO_CONFIG_1, data))
		return ERROR_IMU_GYRO_INIT;

	// 1100 / (1 + 10) ≈ Gyro 100Hz
	data = 10;
	if(ICM20948_WriteReg(GYRO_SMPLRT_DIV, data))
		return ERROR_IMU_GYRO_INIT;

	// Accel: ±2g
	data = (0x00 << 1) | 0x01;
	if(ICM20948_WriteReg(ACCEL_CONFIG, data))
		return ERROR_IMU_ACCEL_INIT;

	// Accel: 100Hz
//	uint16_t acc_div = 11;
	uint16_t acc_div = 10;  // 1125/(1+10) = 102 HZ
	uint8_t high = (acc_div >> 8) & 0xFF;
	uint8_t low  = acc_div & 0xFF;
	if(ICM20948_WriteReg(ACCEL_SMPLRT_DIV_1, high))
		return ERROR_IMU_ACCEL_INIT;
	if(ICM20948_WriteReg(ACCEL_SMPLRT_DIV_2, low))
		return ERROR_IMU_ACCEL_INIT;

	ICM20948_Sel_Bank(0);
	// Disable I2C Master Mode (USER_CTRL Register)
	data = 0x00;
	if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, REG_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 100)) {
		return ERROR_IMU_INIT;
	}
	HAL_Delay(10);

	// set INT pin configuration (INT_PIN_CFG)
	// Bit 7: INT1_ACTL (0: Rising edge, 1: Falling edge)
	// Bit 5: INT1_LATCH_EN (0: 50us pulse, 1: wait until read clear)
	// Bit 4: INT1_ANYRD_2CLEAR (1: read register will clear INT)
	// Bit 1: BYPASS_EN (1: Enable Bypass for Mag)
	data = 0x12; // 0001 0010 (Bypass ON + AnyRead Clear)
  	if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, REG_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100)) {
		return ERROR_I2C;
  	}
	
  	// Enable data ready INT (INT_ENABLE_1)
  	// Bit 0: RAW_DATA_0_RDY_EN (1: Enable)
  	data = 0x01;
  	if(HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR, REG_INT_ENABLE_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100)) {
		return ERROR_I2C;
  	}
  	HAL_Delay(20);

  	// Enable accel + gyro
  	//ICM20948_WriteReg(PWR_MGMT_2, 0x00);  // PWR_MGMT_2

  	// reset Magnetometer (CNTL3 Register)
  	data = 0x01;
  	if(HAL_I2C_Mem_Write(&hi2c1, AK09916_ADDR, 0x32, I2C_MEMADD_SIZE_8BIT, &data, 1, 100)) {
		return ERROR_IMU_MAG_INIT;
  	}
  	HAL_Delay(100);

  	// Magnetometer 100 Hz resolution
  	data = 0x08;
  	if(HAL_I2C_Mem_Write(&hi2c1, AK09916_ADDR, 0x31, I2C_MEMADD_SIZE_8BIT, &data, 1, 100)) {
		return ERROR_IMU_MAG_INIT;
  	}

	// set Data Ready Interrupt
  	//	ICM20948_Sel_Bank(0);
  	//	ICM20948_WriteReg(0x10, 0x01);  // INT_ENABLE_1 (RAW_RDY_EN)
  	acc_scale = 1.0f / ACC_SCALE_2;
  	gyro_scale = 1.0f / GRYO_SCALE_250;
//  	ICM20948.mag_state = MAG_IDLE;
  	is_IMU_init = 1;
  	return OK;
}
