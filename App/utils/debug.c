#include "debug.h"
#include "power_manager.h"
#include "motion_sensor.h"
#include "imu_fusion.h"
#include "i2c.h"
#include "ultrasonic_sensor.h"
#include "storage_control.h"
extern IMU ICM20948;
extern IMU_Data_t imu_data_calibrated;
extern Euler_t euler;
extern Ultrasonic_t front_sonic, rear_sonic;
extern Config_t cur_conf;
extern Config_Record_t last_conf_record;
/*int _write(int32_t file, uint8_t *ptr, int32_t len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}
int fputc(int ch, FILE *f)
{
    ITM_SendChar(ch);
    return ch;
}*/

/* print via SWO
#define PRINT_BUF_SIZE 128
volatile uint8_t uartTxBusy = 0;   // 1 = busying, 0 = standby
char uartBuf[PRINT_BUF_SIZE];

int fputc(int ch, FILE *f)
{
    if(ch == '\n') {
        uint8_t cr = '\r';
        while(uartTxBusy);
        uartTxBusy = 1;
        HAL_UART_Transmit_IT(&huart1, &cr, 1);
    }

    uint8_t c = (uint8_t)ch;
    while(uartTxBusy);
    uartTxBusy = 1;
    HAL_UART_Transmit_IT(&huart1, &c, 1);

    return ch;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) {
        uartTxBusy = 0;
    }
}*/
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

//Debug Unfreeze
void Debug_Unfreeze()
{
	__HAL_DBGMCU_UNFREEZE_TIM8();
	__HAL_DBGMCU_UNFREEZE_TIM6(); 
	//DBGMCU->CR &= ~DBGMCU_CR_DBG_TIM1_STOP; //clear Tim1 Stop
}

void Set_Run_On_Ram()
{
	SCB->VTOR = 0x20000000;
}

//block function for Keil MDS UART
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void Debug_Print()
{
	float vol = Get_Voltage();
	printf("voltage = %lf\r\n", vol);
#if ENABLE_IMU
#if (!ENABLE_IMU_ACC_CALIBRATION)
/*
	ICM20948_WhoAmI();
	uint8_t status = ICM20948_ReadReg(0x1A);  // INT_STATUS_1
	printf("INT_STATUS = 0x%02X\r\n", status);
*/
//		printf("INT_Flag%d|DMA_Busy%d\r\n", ICM20948.INT_Flag ,ICM20948.DMA_Busy);
		printf("AccX:%.2f | AccY:%.2f | AccZ:%.2f\r\n", ICM20948.ax_g, ICM20948.ay_g, ICM20948.az_g);
		printf("calibrated: AccX:%.2f | AccY:%.2f | AccZ:%.2f\r\n", imu_data_calibrated.ax, imu_data_calibrated.ay, imu_data_calibrated.az);
		printf("GyroX:%.2f | GyroY:%.2f | GyroZ:%.2f\r\n",ICM20948.gx_dps, ICM20948.gy_dps, ICM20948.gz_dps);
		printf("calibrated: GyroX:%.2f | GyroY:%.2f | GyroZ:%.2f\r\n", imu_data_calibrated.gx, imu_data_calibrated.gy, imu_data_calibrated.gz);


	  printf("MagX:%.2f | MagY:%.2f | MagZ:%.2f\r\n", ICM20948.Mag_X, ICM20948.Mag_Y, ICM20948.Mag_Z);
	  printf("calibrated: MagX:%.2f | MagY:%.2f | MagZ:%.2f\r\n", imu_data_calibrated.mx, imu_data_calibrated.my, imu_data_calibrated.mz);
	  printf("euler roll:%.2f |  pitch:%.2f | yaw:%.2f\r\n", euler.roll, euler.pitch, euler.yaw);
#endif	//end of (!ENABLE_IMU_ACC_CALIBRATION)
#endif
#if ENABLE_SONIC
	  printf("front Usonic distance:%.2f, rear Usonic distance:%.2f\r\n", front_sonic.distance, rear_sonic.distance);
#endif
#if	ENABLE_EXT_FLASH
	  uint8_t rx[3] = {0};
	  JEDEC_Read(rx, 3);
	  printf("JEDEC ID :%02X %02X %02X \r\n", rx[0], rx[1], rx[2]);
#endif
#if ENABLE_CONF
	  Get_Cur_Conf(&cur_conf);
	  printf("conf ver:%x , state:%x , counter:%x\r\n", cur_conf.version, cur_conf.state,  cur_conf.counter);
	  printf("conf accel: %.2f , gyro: %.2f , mag: %.2f\r\n", last_conf_record.imu_offset[0], last_conf_record.imu_offset[1],
			  last_conf_record.imu_offset[2]);
	  printf("conf pid_kp : %f ,pid_ki : %f ,pid_kd : %f \r\n ",
			  last_conf_record.pid_kp,
			  last_conf_record.pid_ki,
			  last_conf_record.pid_kd);
#endif
}

void I2C__Dev_Discovery(I2C_HandleTypeDef *hi2c){
	for (uint8_t i = 1; i < 128; i++) {
		if (HAL_I2C_IsDeviceReady(hi2c, i << 1, 1, 100) == HAL_OK) {
			printf("Found Device at Address: 0x%02X\n", i << 1);
		}
	}
}
