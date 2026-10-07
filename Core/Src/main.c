/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Motor.h"
#include "debug.h"
#include "power_manager.h"
#include "motion_sensor.h"
#include "imu_fusion.h"
#include "servo.h"
#include "vl53l1_platform.h"
#include "ultrasonic_sensor.h"
#include "storage_control.h"
#include "comm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	SYS_INIT = 1000,
	SYS_IDLE = 1001,
	SYS_AUTO_RUN = 1002,
	SYS_MANUALLY_RUN = 1003,
	SYS_ERROR = 1004,
	SYS_LOW_POWER = 1005
}SystemState;

typedef enum{
    RUN_IDLE = 5000,
    RUN_FORWARD = 5001,
    RUN_TURN_LEFT = 5002,
    RUN_TURN_RIGHT = 5003,
    RUN_AVOID = 5004,
    RUN_STOP =5005
}RunState;

typedef enum {
    EVENT_NONE = 7000,
    EVENT_OBSTACLE = 7001,
    EVENT_OBSTACLE_CLEARANCE = 7002,
    EVENT_ERROR = 7003
}SystemEvent;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DOF_RETRY_CNT 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t Error_code;
Encoder_t rEncoder, lEncoder;
//extern uint8_t print_flag;
float voltage = 0;
float filtered_voltage = 0;
enum power_status PowerStatus;
volatile SystemState sState;
volatile RunState rState;
volatile SystemEvent sEvent;

volatile uint8_t start_button; //not ready

volatile uint8_t flag_5ms = 0;
volatile uint8_t flag_10ms = 0;
volatile uint8_t flag_100ms = 0;
volatile uint8_t flag_1s = 0;

PID_t pid_L = {0.8, 0.004, 0, 0, 0}; // left motor PID
PID_t pid_R = {0.8, 0, 0, 0, 0}; //right motor PID

IMU ICM20948;
volatile CalibState_t calib_state;
IMU_Data_t imu_data_calibrated;
Euler_t euler;
uint8_t is_IMU_init;
#if ENABLE_IMU_ACC_CALIBRATION
	volatile AccCalibState_t acc_cal_state;
#endif
//Servo
Servo_PID_t Servo_PID = {1.2f, 0.3f, 0, 0, 0, 0.01}; //servo PID

//TOF
volatile uint8_t TOF_interrupt;
VL53L1_Dev_t RR_TOF, RL_TOF, FR_TOF, FL_TOF;

//ultrasonic sensor
volatile SonicState_t sonic_state = TRIG_FRONT;
Ultrasonic_t front_sonic, rear_sonic;
//log & config
uint64_t time_offset_us;
Config_t cur_conf;
Config_Record_t last_conf_record;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void System_Task();
void Move_Task();
void ICM20948_Recover_Bus();
void I2C_Bus_Recover();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  sState = SYS_INIT;
  rState = RUN_IDLE;
#if ENABLE_IMU_ACC_CALIBRATION
  acc_cal_state = CAL_DONE;
#endif
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  I2C_Bus_Recover();	//for ICM20948 hi2c1
  HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_CRC_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  Power_Manager_Init();
#if ENABLE_EXT_FLASH
  FLASH_Config_Init();
  //last_conf_record.imu_offset[0] = 1;
  //FLASH_Config_Write(&last_conf_record);
#endif
#if ENABLE_IMU
  HAL_Delay(100);
  Error_code = ICM20948_Init();
  HAL_Delay(100);
  uint8_t retry = 0;
  while(ICM20948_WhoAmI() == 0 || Error_code != OK)
  {
#if DEBUG_MODE
	  printf("ErrorCode:%d\r\n", Error_code);
#endif
	  HAL_Delay(100);
	  I2C_Bus_Recover();
	  HAL_Delay(100);
	  ICM20948_Recover_Bus(&hi2c1);
	  HAL_Delay(500);
	  Error_code = ICM20948_Init();
	  retry++;
	  if( retry == DOF_RETRY_CNT)
	  {
		  sState = SYS_ERROR;
		  break;
	  }
  };
  IMU_Fusion_Init();
#else
  calib_state = CALIB_DONE;
#endif
  Motor_Init();
  Servo_Init();
  HAL_TIM_Base_Start_IT(&htim6); //Start System interrupts on Timer 6
#if ENABLE_SONIC
  Error_code = Ultrasonic_Init(US_HTIM_HANDLE);
#endif

#if ENABLE_TOF
  Error_code = VL53L1_Init_All();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	Mortor_Controler(&htim8, TIM_CHANNEL_3, FORWORD, 100);
//	Mortor_Controler(&htim8, TIM_CHANNEL_4, FORWORD, 100);
//	Mortor_Controler(&htim8, TIM_CHANNEL_3, STOP, 0);
//	Mortor_Controler(&htim8, TIM_CHANNEL_4, STOP, 0);
#if DEBUG_MODE
//  Debug_Unfreeze();
  sEvent = EVENT_OBSTACLE_CLEARANCE;
  printf("Start running\r\n");
#endif
  if(Error_code)
  {
	  sState = SYS_ERROR;
  }
  while (1)
  {
#if ENABLE_TOF
	  if(TOF_interrupt)
		  Get_TOF_Distance();
#endif
#if ENABLE_SONIC
	  Ultrasonic_Task(US_HTIM_HANDLE);
#endif
	  if(flag_5ms)
	  {
#if ENABLE_IMU_ACC_CALIBRATION
#endif
		  flag_5ms = 0;
	  }
	  if(flag_1s)
	  {
		  Power_Manager();
		  flag_1s = 0;
#if DEBUG_MODE
		  Debug_Print();
#endif
	  }
	  if(ICM20948.INT_Flag && !ICM20948.DMA_Busy)
	  {
		  Error_code = IMU_Read_DMA();
	//		printf("ERROR Code:%d | HAL ERROR Code:%d\r\n", Error_code, hi2c1.ErrorCode);
	  }
	  if(ICM20948.DMA_Done || ICM20948.Mag_DMA_Done)
	  {
		  ICM20948_Parse();
		  if(sState == SYS_INIT)
			  IMU_Calibrate_Task();
		  if(calib_state == CALIB_DONE)
		  {
			  IMU_Fusion_Update();
	//			flag_10ms=0;
		//		IMU_Fusion_Update_v2();
		  }
		  if(ICM20948.Mag_DMA_Done)	ICM20948.Mag_DMA_Done = 0;
		  if(ICM20948.DMA_Done)		ICM20948.DMA_Done = 0;
#if DEBUG_MODE
	//		printf("2---ERROR Code:%d | HAL ERROR Code:%d\r\n", Error_code, hi2c1.ErrorCode);
#endif
	  }
	  System_Task();
	  if(sState == SYS_AUTO_RUN)
		  Move_Task();
	  else continue;
		// HAL_Delay(100);
		/*Mortor_Controler(&htim8, TIM_CHANNEL_3, FORWORD, 50);
		Mortor_Controler(&htim8, TIM_CHANNEL_4, FORWORD, 50);
		printf("R diff=%d rpm=%.2f | L diff=%d rpm=%.2f\r\n",
               rEncoder.diff, rEncoder.rpm_filtered,
               lEncoder.diff, lEncoder.rpm_filtered);
		HAL_Delay(3000);

		Mortor_Controler(&htim8, TIM_CHANNEL_3, STOP, 0);
		Mortor_Controler(&htim8, TIM_CHANNEL_4, STOP, 0);
		printf("R diff=%d rpm=%.2f | L diff=%d rpm=%.2f\r\n",
               rEncoder.diff, rEncoder.rpm_filtered,
               lEncoder.diff, lEncoder.rpm_filtered);
		HAL_Delay(3000);

		Mortor_Controler(&htim8, TIM_CHANNEL_3, REVERSES, 100);
		Mortor_Controler(&htim8, TIM_CHANNEL_4, REVERSES, 100);
		printf("R diff=%d rpm=%.2f | L diff=%d rpm=%.2f\r\n",
               rEncoder.diff, rEncoder.rpm_filtered,
               lEncoder.diff, lEncoder.rpm_filtered);
		HAL_Delay(3000);*/
		/*HAL_Delay(10);
			printf("diff=%d\r\n", rEncoder.diff);*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*
* INIT | IDLE | RUN | LOW_POWER | ERROR
*/
void System_Task(){
	switch(sState)
	{
		case SYS_INIT:
			Stop_Motor();
		    if (calib_state == CALIB_DONE)
		    	sState = SYS_IDLE;
		break;
		case SYS_IDLE:
			Stop_Motor();
			//if(start_button)
			sState = SYS_AUTO_RUN;
		break;
		case SYS_AUTO_RUN:
			if(flag_10ms)
			{
				//Servo PID
				Servo_PID_Update(&htim4, TIM_CHANNEL_2);
				float corr = 0;
				float target_L = 0;
				float target_R = 0;
				//Get RPM
				Update_Encoder(&lEncoder);
				Update_Encoder(&rEncoder);
				float base_speed = pid_L.target_RPM;
				//如果要轉彎,後輪差速控制
				Differential_Control(base_speed, &target_L, &target_R);
				//Motor PID
				if( fabs(Servo_PID.last_output) < 5.0f)
				{
					corr = Straight_Correction(lEncoder.reduction_rpm, rEncoder.reduction_rpm);
					target_L = pid_L.target_RPM - corr;
					target_R = pid_R.target_RPM + corr;
				}
				//轉彎降速
				float scale = 1.0f - fabs(Servo_PID.last_output)/30.0f;
				if(scale < 0.4f) scale = 0.4f;
				target_L *= scale;
				target_R *= scale;
				if(target_L < 0) target_L = 0;
				if(target_R < 0) target_R = 0;
				if(target_L > MAX_RPM) target_L = MAX_RPM;
				if(target_R > MAX_RPM) target_R = MAX_RPM;
				PID_Update(&pid_L, target_L, lEncoder.reduction_rpm);
				PID_Update(&pid_R, target_R, rEncoder.reduction_rpm);
				Set_Pwm_Duty_Percent(&htim8, TIM_CHANNEL_3, pid_L.last_output);
				Set_Pwm_Duty_Percent(&htim8, TIM_CHANNEL_4, pid_R.last_output);
#if DEBUG_MODE
		//			printf("R RPM=%.2f| L RPM=%.2f\r\n",
		//						rEncoder.reduction_rpm,lEncoder.reduction_rpm);
#endif
				flag_10ms = 0;
			}
			if(PowerStatus == PROTECTION_POWER_STATUS)
				sState = SYS_LOW_POWER;
			else if(sEvent == EVENT_ERROR)
				sState = SYS_ERROR;
		break;
		case SYS_MANUALLY_RUN://manually run

		break;
		case SYS_LOW_POWER://slow down or stop

		break;
		case SYS_ERROR://stop
			printf("ERROR Code:%d | HAL ERROR Code:%d\r\n", Error_code, (int)hi2c1.ErrorCode);
			Stop_Motor();
			HAL_Delay(5000);
		break;
	}
}

void Move_Task()
{
	switch(rState)
	{
		case RUN_IDLE:
			Stop_Motor();
#if DEBUG_MODE
			sEvent = EVENT_OBSTACLE_CLEARANCE;
#endif
			rState = RUN_FORWARD;
//			rState = RUN_TURN_LEFT;//for test servo
        break;
		case RUN_FORWARD:
			if(sEvent == EVENT_OBSTACLE)
				rState = RUN_AVOID;
			if(sEvent == EVENT_OBSTACLE_CLEARANCE)
			{
				Mortor_Controler(&htim8, TIM_CHANNEL_3, FORWORD, 100);
				Mortor_Controler(&htim8, TIM_CHANNEL_4, FORWORD, 100);
			}
        break;
		case RUN_AVOID:
        	if(sEvent == EVENT_OBSTACLE_CLEARANCE)
        		rState = RUN_FORWARD;
        break;
		case RUN_TURN_LEFT:
			Mortor_Controler(&htim8, TIM_CHANNEL_3, RIGHT, 30);
			Mortor_Controler(&htim8, TIM_CHANNEL_4, RIGHT, 30);
			Set_Servo_Angle(&htim4, TIM_CHANNEL_2, 30);
        break;
		case RUN_TURN_RIGHT:
            Mortor_Controler(&htim8, TIM_CHANNEL_3, RIGHT, 20);
			Mortor_Controler(&htim8, TIM_CHANNEL_4, RIGHT, 20);
        break;
		case RUN_STOP:

        break;
	}
}

//External interrupts callback function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_6) //ICM20948 INT
	{
		ICM20948.INT_Flag = 1;
	}
    if(GPIO_Pin == GPIO_PIN_2)//front right dof
    {
		TOF_interrupt |= FRONT_RIGHT_TOF_FLAG;
    }
    if(GPIO_Pin == GPIO_PIN_3)//front left dof
    {
    	TOF_interrupt |= FRONT_LEFT_TOF_FLAG;
    }
	if(GPIO_Pin == GPIO_PIN_14)//Rear right dof
    {
		TOF_interrupt |= REAR_RIGHT_TOF_FLAG;
    }
    if(GPIO_Pin == GPIO_PIN_15)//Rear left dof
    {
    	TOF_interrupt |= REAR_LEFT_TOF_FLAG;
    }
}

void EXTI_Callback()
{
	ICM20948.INT_Flag = 0;
}

//DMA callback function
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1)
	{
		if(ICM20948.read_mag_cnt > 5)
		{
//			if(ICM20948.mag_state == MAG_READ_DATA)
//			{
				ICM20948.read_mag_cnt = 0;
				ICM20948.Mag_DMA_Done = 1;
//			}
		}
		else
			ICM20948.DMA_Done = 1;

		ICM20948.DMA_Busy = 0;
	}
}

//DMA Error
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
	  ICM20948.DMA_Busy = 0;	//unlock & read again
  }
}

void ICM20948_Recover_Bus(I2C_HandleTypeDef *hi2c)
{
    __HAL_I2C_DISABLE(hi2c);
    HAL_Delay(10);
    // reset I2C hardware
    if (HAL_I2C_Init(hi2c) != HAL_OK) {
        //init error
    }
    //__HAL_I2C_ENABLE(hi2c);
}

void I2C_Bus_Recover(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // 1. set SCL and SDA as GPIO output mode
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;// PB8, PB9
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // OpenDrain
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // pull high (release bus)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // SCL
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // SDA
    Delay_us(5);

    // 2. send 9 pulse manually
    for(int i = 0; i < 9; i++) {
    	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
    	{
    		Delay_us(5);
    		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
    			break;
    	}
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    	Delay_us(5);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    	Delay_us(5);
    }

    // 3. send STOP signal (STOP condition)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    Delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    Delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    Delay_us(5);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
