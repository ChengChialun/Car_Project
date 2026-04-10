/* Includes ------------------------------------------------------------------*/
#include "Motor.h"
static uint32_t arr;//AUTORELOAD
extern PID_t pid_L, pid_R;
void Set_Pwm_Duty_Percent(TIM_HandleTypeDef *tim, unsigned int channel, float percent)
{
	if(channel == TIM_CHANNEL_3)	pid_L.target_RPM = MAX_RPM * percent / 100.0f;
	else if(channel == TIM_CHANNEL_4)	pid_R.target_RPM = MAX_RPM * percent / 100.0f;
//    uint32_t duty = percent/100.0f * (arr + 1);
	float limit = Get_Power_Limit();
	float final_percent = percent * limit;

	if (final_percent > 0 && final_percent < MIN_PERCENT && limit > 0) //avoid PWM too low 
		final_percent = MIN_PERCENT;
	
	uint32_t duty = (uint32_t)(final_percent * arr/100.0f);  //0~100
	
	__HAL_TIM_SET_COMPARE(tim, channel, duty);
}

void Mortor_Controler(TIM_HandleTypeDef *tim, unsigned int channel, enum Direction dir, float percent)
{
	switch(dir)
	{
		case FORWORD:
		case LEFT:
		case RIGHT:
			if(channel == TIM_CHANNEL_3)
			{
				HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);//1
				HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);//0
			}
			else if(channel == TIM_CHANNEL_4)
			{
				HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);//1
				HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);//0
			}
		break;
		case REVERSES:
			if(channel == TIM_CHANNEL_3)
			{
				HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);//0
				HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);//1
			}
			else if(channel == TIM_CHANNEL_4)
			{
				HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);//0
				HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);//1
			}
		break;
		case SHORT_BRAKE:			
			HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);//1
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);//1
			HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);//1
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);//1
		break;
		default://STOP, STANDBY
			HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);//0
			HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);//0
			HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);//0
			HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);//0
		break;
	}

	if(dir == STANDBY)//Enable STBY
		HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);//Pull GPIO A5 as low
	else
		HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);//Pull GPIO A5 as High

	Set_Pwm_Duty_Percent(tim, channel, percent);
}

void Stop_Motor()
{
	Reset_PID(&pid_L);
	Reset_PID(&pid_R);
	Mortor_Controler(&htim8, TIM_CHANNEL_3, STOP, 0);
	Mortor_Controler(&htim8, TIM_CHANNEL_4, STOP, 0);
}

void Motion_Controler(TIM_HandleTypeDef *tim, unsigned int channel, enum Direction dir, float percent)
{
}

void Update_Encoder(Encoder_t *enc)
{
  int16_t now = __HAL_TIM_GET_COUNTER(enc->htim);
  int16_t diff = now - enc->last_count;
	if(enc == &rEncoder)
		diff = -diff;
	enc->diff = diff;
  enc->last_count = now;

  float rpm = (diff / (float)PULSE_PER_REV) * (60.0f/SAMPLE_TIME);
	
	if(diff == 0)
		enc->rpm_filtered *= 0.9f;
	else//Low-Pass Filter
		enc->rpm_filtered = enc->rpm_filtered * 0.7f + rpm * 0.3f;
	
	if(fabs(enc->rpm_filtered) < 0.01f)
		enc->rpm_filtered = 0.0f;
	
	//avoid overflow
	if(enc->rpm_filtered > 20000) enc->rpm_filtered = 20000;
	if(enc->rpm_filtered < -20000) enc->rpm_filtered = -20000;
	
	enc->reduction_rpm = enc->rpm_filtered/REDUCTION_RATIO;
	return;
}

void Motor_Init()
{
	//Enable STBY
	HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);//Pull GPIO A5 as High
	//CW
	/*HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);//1
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);//0
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);//1
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);//0*/
	//Enable PWM
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	__HAL_TIM_MOE_ENABLE(&htim8);
	arr = __HAL_TIM_GET_AUTORELOAD(&htim8); //get AUTORELOAD 

	//Enable Hall Encoder on Tim2, Tim3
	//Tim2(left rear)PA0, PA1
	//Tim3(right rear)PA6, PA7
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	lEncoder.htim = &htim2;
	rEncoder.htim = &htim3;
}

void Reset_PID(PID_t *pid)
{
	pid->integral = 0;
	pid->last_deviation = 0;
}

void PID_Update(PID_t *pid, float target_rpm, float current_rpm)
{
	if(fabs(target_rpm) < 0.01f)
	{
		Reset_PID(pid);
		return;
	}
	float deviation = target_rpm - current_rpm;
	//Deadband
	if(fabs(deviation) < 3)
		return;

	//計算累積誤差
	pid->integral += deviation * 0.01f; // 0.01s = 10ms
	
	//Anti-windup
	if(pid->integral > 100) pid->integral = 100;
	if(pid->integral < -100) pid->integral = -100;

	//float derivative = (deviation - last_deviation) / 0.01f; the same below
	float derivative = (deviation - pid->last_deviation) * 100.0f; //more fast in MCU
	//PID output
	float output = pid->Kp * deviation + pid->Ki * pid->integral + pid->Kd * derivative;
    
	// limitation PWM 0~100
	if(output > 100) output = 100;
	if(output < 0) output = 0;
	pid->last_deviation = deviation;
	pid->last_output = output;
	// PWM
	return;
}

float Straight_Correction(float rpm_L, float rpm_R)
{
	float diff = rpm_L - rpm_R;
	return diff*K_SYNC;
}
